import asyncio
import struct
import math
import time
import ms5837
import numpy as np
from enum import IntEnum
from SPIContainer import SPI_Xfer_Container
from HROVcontrolsystem import ControlSystem
from HROVcontrolsystem import Axes
from asynctimer import AsyncTimer
from thruster import Thrusters
from navx import Navx
from ligths import Lights
from servo import Servo
from utils import constrain, map_value
from async_hiwonder_reader import AsyndHiwonderReader
import time
import copy
from MultiaxisManipulator import MultiaxisManipulator, GripState

to_rad = math.pi / 180

UDP_FLAGS_MASTER = np.uint64(1 << 0)
UDP_FLAGS_LIGHT_STATE = np.uint64(1 << 1)
UDP_FLAGS_STAB_ROLL = np.uint64(1 << 2)
UDP_FLAGS_STAB_PITCH = np.uint64(1 << 3)
UDP_FLAGS_STAB_YAW = np.uint64(1 << 4)
UDP_FLAGS_STAB_DEPTH = np.uint64(1 << 5)
UDP_FLAGS_RESET_POSITION = np.uint64(1 << 6)
UDP_FLAGS_RESET_IMU = np.uint64(1 << 7)
UDP_FLAGS_UPDATE_PID = np.uint64(1 << 8)
UDP_FLAGS_MANIPULATOR_ANGLE_1_UPDATE = np.uint64(1 << 9)
UDP_FLAGS_MANIPULATOR_ANGLE_2_UPDATE = np.uint64(1 << 10)
UDP_FLAGS_MANIPULATOR_ANGLE_3_UPDATE = np.uint64(1 << 11)
UDP_FLAGS_MANIPULATOR_GRIP_UPDATE = np.uint64(1 << 12)

YAW_CAP = 0.3

class IMUType(IntEnum):
    POLOLU = 0
    NAVX = 1
    HIWONDER = 2

class ControlType(IntEnum):
    DIRECT_CTRL = 0
    STM_CTRL = 1

class UDPRxValues(IntEnum):
    FLAGS = 0
    FORWARD = 1
    STRAFE = 2
    VERTICAL = 3
    ROTATION = 4
    ROLL_INC = 5
    PITCH_INC = 6
    POWER_TARGET = 7
    CAM_ROTATE = 8
    MAN_GRIP = 9
    MAN_ROTATE = 10
    ROLL_KP = 11
    ROLL_KI = 12
    ROLL_KD = 13
    PITCH_KP = 14
    PITCH_KI = 15
    PITCH_KD = 16
    YAW_KP = 17
    YAW_KI = 18
    YAW_KD = 19
    DEPTH_KP = 20
    DEPTH_KI = 21
    DEPTH_KD = 22
    MANIPULATOR_ANGLE_1 = 23
    MANIPULATOR_ANGLE_2 = 24
    MANIPULATOR_ANGLE_3 = 25
    MANIPULATOR_GRIP = 26

class RemoteUdpDataServer(asyncio.Protocol):
    def __init__(self, contolSystem: ControlSystem, timer: AsyncTimer, imuType: IMUType, controlType: ControlType, manipulator: MultiaxisManipulator,
                  bridge: SPI_Xfer_Container = None, navx: Navx = None, thrusters: Thrusters = None,
                  lights: Lights = None, cameraServo: Servo = None, hiwonderReader: AsyndHiwonderReader = None):
        self.controlType = controlType
        self.hiwonderReader = hiwonderReader
        self.imuType = imuType
        self.timer = timer
        self.bridge = bridge        
        self.controlSystem = contolSystem
        self.navx = navx
        self.thrusters = thrusters
        self.lights = lights
        self.cameraServo = cameraServo
        self.remoteAddres = None
        self.manipulator = manipulator
        self.timer.subscribe(self.dataCalculationTransfer)
        self.timer.start()
        if self.hiwonderReader is not None:
            self.hiwonderReader.start()
        self.powerTarget = 0
        self.cameraRotate = 0
        self.cameraAngle = 65
        self.lightState = 0
        self.eulers = [0.0, 0.0, 0.0]
        self.accelerations = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.IMURaw = [0.0, 0.0, 0.0]
        self.eulerMag = [0.0, 0.0, 0.0]
        self.voltage = 0
        self.curAll = 0
        self.curLights = [0.0, 0.0]
        self.depth = 0
        self.MASTER = True
        self.IMUErrors = [0.0, 0.0, 0.0]
        self.incrementScale = 0.5
        self.batCharge = 0
        self.resetIMU = 0
        self.TelemetryFlags = np.uint64(0)
        self.controlFlags = np.uint64(0)
        self.masterChangedToSafe = False

        self.maxPowerTarget = 1.0
        
        self.newRxPacket = True
        self.newTxPacket = True
        if self.newRxPacket:
            self.MASTER = False
        
        self.depthDelay = 10
        self.counter = 0

        self.manControl = False
        self.manTelemetryObtained = False

        self.time1 = time.time()
        self.time2 = time.time()
        self.thrustersPhaseCurrents = [[0.0]*3]*6

        if self.imuType == IMUType.NAVX:
            navx.subscribe(self.navx_data_received)
        
        try:
            self.depth_sensor = ms5837.MS5837(model=ms5837.MODEL_30BA, bus=1)
            self.depth_sensor.init()
        except Exception:
            print('Depth sensor init failed')
            self.ds_init = 0
        else:
            self.ds_init = 1
            print('Depth sensor init complete')        
        time.sleep(2)
        print('Ready to drown!')

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, address):
        packet = data
        if len(packet) == 2 and packet[0] == 0xAA and packet[1] == 0xFF:
            self.remoteAddres = address
            print(f"Client {address} connected")
            return
        
        if not self.remoteAddres:
            return
        
        if not self.newRxPacket:            
            #fx, fy, vertical_thrust, powertarget, rotation_velocity, manipulator_grip, manipulator_rotate, camera_rotate, reset, light_state, stabilization, RollInc, PitchInc, ResetPosition.
            received = struct.unpack_from("=ffffffffBBBffBffffffffffffB", packet)
            self.powerTarget = received[3] * self.maxPowerTarget
            
            self.controlSystem.setAxisInput(Axes.STRAFE, (received[0] ** 3) * 100 * self.powerTarget)
            self.controlSystem.setAxisInput(Axes.FORWARD, (received[1] ** 3) * 100 * self.powerTarget)
            self.controlSystem.setAxisInput(Axes.DEPTH, (received[2] ** 3) * 100 * self.powerTarget)
            self.controlSystem.setAxisInput(Axes.YAW, (received[4] ** 3) * 100 * self.powerTarget) 

            self.cameraRotate = received[7]
            self.cameraAngle += self.cameraRotate * self.incrementScale * 3
            self.cameraAngle = constrain(self.cameraAngle, 40, 90)
            self.lightState = received[9]

            rollStab =  1 if received[10] & 0b00000001 else 0
            pitchStab = 1 if received[10] & 0b00000010 else 0
            yawStab =   1 if received[10] & 0b00000100 else 0
            depthStab = 1 if received[10] & 0b00001000 else 0

            self.controlSystem.setStabilization(Axes.ROLL, rollStab)
            self.controlSystem.setStabilization(Axes.PITCH, pitchStab)
            self.controlSystem.setStabilization(Axes.YAW, yawStab)
            self.controlSystem.setStabilization(Axes.DEPTH, depthStab)
            
            if received[11]: 
                rollSP = self.controlSystem.getPIDSetpoint(Axes.ROLL) + received[11] * self.incrementScale
                self.controlSystem.setPIDSetpoint(Axes.ROLL, rollSP)
            if received[12]: 
                pitchSP = self.controlSystem.getPIDSetpoint(Axes.PITCH) + received[12] * self.incrementScale
                self.controlSystem.setPIDSetpoint(Axes.PITCH, pitchSP)
            
            if(received[13]):
                self.controlSystem.setPIDSetpoint(Axes.ROLL, 0)
                self.controlSystem.setPIDSetpoint(Axes.PITCH, 0)
                self.controlSystem.setPIDSetpoint(Axes.YAW, self.controlSystem.getAxisValue(Axes.YAW))
                self.controlSystem.setPIDSetpoint(Axes.DEPTH, self.controlSystem.getAxisValue(Axes.DEPTH))
                self.controlSystem.setStabilizations([0,0,0,0,0,0])
                
            if(received[26]):
                self.controlSystem.setPIDConstants(Axes.ROLL, [received[14], received[15], received[16]])
                self.controlSystem.setPIDConstants(Axes.PITCH, [received[17], received[18], received[19]])
                self.controlSystem.setPIDConstants(Axes.YAW, [received[20], received[21], received[22]])
                self.controlSystem.setPIDConstants(Axes.DEPTH, [received[23], received[24], received[25]])
        else:
            # controlFlags, forward, strafe, vertical, rotation, rollInc, pitchInc, powerTarget, cameraRotate, manipulatorGrip, 
            # manipulatorRotate, rollKp, rollKi, rollKd, pitchKp, pitchKi, pitchKd, yawKp, yawKi, yawKd, depthKp, depthKi, depthKd,
            # Extended packet:
            # manipulatorAngle1, manipulatorAngle2, manipulatorAngle3, manipulatorGrip
            # flags = MASTER, lightState, stabRoll, stabPitch, stabYaw, stabDepth, resetPosition, resetIMU, updatePID, 
            # Extended packet:
            # manipulatorAngleUpdate1, manipulatorAngleUpdate2, manipulatorAngleUpdate3, manipulatorGripUpdate

            received = None
            if len(packet) == 96:
                received = struct.unpack_from("=Qffffffffffffffffffffff", packet)                    
                self.manControl = False
            if len(packet) == 109:
                received = struct.unpack_from("=QfffffffffffffffffffffffffB", packet)
                #print(*["%.2f" % elem for elem in received], sep ='; ')
                self.manControl = True
            self.controlFlags = np.uint64(received[UDPRxValues.FLAGS])
            self.MASTER = self.controlFlags & UDP_FLAGS_MASTER
            self.lightState = self.controlFlags & UDP_FLAGS_LIGHT_STATE
            rollStab =  self.controlFlags & UDP_FLAGS_STAB_ROLL
            pitchStab = self.controlFlags & UDP_FLAGS_STAB_PITCH
            yawStab =   self.controlFlags & UDP_FLAGS_STAB_YAW
            depthStab = self.controlFlags & UDP_FLAGS_STAB_DEPTH
            resetPosition = self.controlFlags & UDP_FLAGS_RESET_POSITION
            self.resetIMU = self.controlFlags & UDP_FLAGS_RESET_IMU
            updatePID = self.controlFlags & UDP_FLAGS_UPDATE_PID
            if self.manControl:
                manipulatorAngleUpdate1 = self.controlFlags & UDP_FLAGS_MANIPULATOR_ANGLE_1_UPDATE
                manipulatorAngleUpdate2 = self.controlFlags & UDP_FLAGS_MANIPULATOR_ANGLE_2_UPDATE
                manipulatorAngleUpdate3 = self.controlFlags & UDP_FLAGS_MANIPULATOR_ANGLE_3_UPDATE
                manipulatorGripUpdate   = self.controlFlags & UDP_FLAGS_MANIPULATOR_GRIP_UPDATE
                self.manipulator.setControlFlags([manipulatorAngleUpdate1,
                                                    manipulatorAngleUpdate2,
                                                    manipulatorAngleUpdate3,
                                                    manipulatorGripUpdate])
                self.manipulator.setControlAngleAll([received[UDPRxValues.MANIPULATOR_ANGLE_1],
                                                    received[UDPRxValues.MANIPULATOR_ANGLE_2],
                                                    received[UDPRxValues.MANIPULATOR_ANGLE_3]])
                self.manipulator.setGripState(received[UDPRxValues.MANIPULATOR_GRIP])
            self.controlSystem.setStabilization(Axes.ROLL, rollStab)
            self.controlSystem.setStabilization(Axes.PITCH, pitchStab)
            self.controlSystem.setStabilization(Axes.YAW, yawStab)
            self.controlSystem.setStabilization(Axes.DEPTH, depthStab) 

            self.powerTarget = received[UDPRxValues.POWER_TARGET] 
            
            self.controlSystem.setAxisInput(Axes.FORWARD, (received[UDPRxValues.FORWARD] ** 3) * 100 * self.powerTarget * self.maxPowerTarget)
            self.controlSystem.setAxisInput(Axes.STRAFE, (received[UDPRxValues.STRAFE] ** 3) * 100 * self.powerTarget * self.maxPowerTarget)
            self.controlSystem.setAxisInput(Axes.DEPTH, (received[UDPRxValues.VERTICAL] ** 3) * 100 * self.powerTarget * self.maxPowerTarget)
            self.controlSystem.setAxisInput(Axes.YAW, (received[UDPRxValues.ROTATION]) * 100 * self.powerTarget * YAW_CAP)

            rollInc = received[UDPRxValues.ROLL_INC]
            pitchInc = received[UDPRxValues.PITCH_INC]

            if rollInc:
                rollSP = self.controlSystem.getPIDSetpoint(Axes.ROLL) + rollInc * self.incrementScale
                self.controlSystem.setPIDSetpoint(Axes.ROLL, rollSP)
            if pitchInc:
                pitchSP = self.controlSystem.getPIDSetpoint(Axes.PITCH) + pitchInc * self.incrementScale
                self.controlSystem.setPIDSetpoint(Axes.PITCH, pitchSP)
                
            self.cameraRotate = received[UDPRxValues.CAM_ROTATE]            
            self.cameraAngle += self.cameraRotate * self.incrementScale
            
            if updatePID:
                self.controlSystem.setPIDConstants(Axes.ROLL, [received[UDPRxValues.ROLL_KP], received[UDPRxValues.ROLL_KI], received[UDPRxValues.ROLL_KD]])
                self.controlSystem.setPIDConstants(Axes.PITCH, [received[UDPRxValues.PITCH_KP], received[UDPRxValues.PITCH_KI], received[UDPRxValues.PITCH_KD]])
                self.controlSystem.setPIDConstants(Axes.YAW, [received[UDPRxValues.YAW_KP], received[UDPRxValues.YAW_KI], received[UDPRxValues.YAW_KD]])
                self.controlSystem.setPIDConstants(Axes.DEPTH, [received[UDPRxValues.DEPTH_KP], received[UDPRxValues.DEPTH_KI], received[UDPRxValues.DEPTH_KD]])
            
            if resetPosition:
                self.controlSystem.setPIDSetpoint(Axes.ROLL, 0)
                self.controlSystem.setPIDSetpoint(Axes.PITCH, 0)
                self.controlSystem.setPIDSetpoint(Axes.YAW, self.controlSystem.getAxisValue(Axes.YAW))
                self.controlSystem.setPIDSetpoint(Axes.DEPTH, self.controlSystem.getAxisValue(Axes.DEPTH))
                self.controlSystem.setStabilizations([0,0,0,0,0,0])
            
            if self.resetIMU:
                self.IMUErrors = copy.copy(self.eulers)
                    
        self.controlSystem.setdt(self.timer.getInterval())

    def navx_data_received(self, sender, data):
        roll, pitch, yaw, heading = data
        self.eulers = [roll, -pitch, yaw]
        

    def dataCalculationTransfer(self):
        self.time2 = time.time()
        #print("%.4f"%(self.time2-self.time1))
        self.time1 = self.time2

        self.counter += 1
        if self.counter >= self.depthDelay:
            self.counter = 0
            if self.ds_init:
                if self.depth_sensor.read(ms5837.OSR_8192):
                    self.depth = self.depth_sensor.pressure(ms5837.UNITS_atm)*10-10

        thrust = self.controlSystem.getThrustersControls()

        #print(*["%.2f" % elem for elem in thrust], sep ='; ')    

        if self.MASTER:
            self.masterChangedToSafe = False
            if self.controlType == ControlType.STM_CTRL:
                self.bridge.set_cam_angle_value(self.cameraAngle)
                lightsValues = [50*self.lightState, 50*self.lightState]
                self.bridge.set_lights_values(lightsValues)            
                self.bridge.set_mots_values(thrust)
                if self.manTelemetryObtained  and self.manControl:
                    manControlFlags = self.manipulator.getControlFlags()
                    for index in range(self.manipulator.getAxesNumber()):
                        if manControlFlags[index]:
                            self.bridge.set_man_angle_value(index,
                                                            self.manipulator.getAxisControlAngle(index))
                            
                        if manControlFlags[3]:
                            self.bridge.set_man_grip_value(self.manipulator.getGripState())
                        self.manipulator.setControlFlags([False]*4)

                if self.controlType == ControlType.DIRECT_CTRL:
                    self.thrusters.set_thrust_all(thrust)
                    if self.lightState:
                        self.lights.on()
                    else:
                        self.lights.off()
            else:
                if not self.masterChangedToSafe:
                    self.masterChangedToSafe = True
                    thrust = [0.0]*6
                    if self.controlType == ControlType.DIRECT_CTRL:
                        self.thrusters.set_thrust_all(thrust)
                        self.lights.off()
                        self.cameraServo.rotate(self.cameraAngle)
                    if self.controlType == ControlType.STM_CTRL:
                        self.bridge.set_cam_angle_value(self.cameraAngle)
                        lightsValues = [0, 0]
                        self.bridge.set_lights_values(lightsValues)            
                        self.bridge.set_mots_values(thrust)
                        if not self.manipulator.getGripState() == GripState.UWMANIPULATOR_GRIP_STOP:
                            self.bridge.set_man_grip_value(GripState.UWMANIPULATOR_GRIP_STOP)
                            self.manipulator.setGripState(GripState.UWMANIPULATOR_GRIP_STOP)

        if self.imuType == IMUType.HIWONDER:
            self.eulers = self.hiwonderReader.getIMUAngles()
        if self.controlType == ControlType.STM_CTRL:
            try:
                # Transfer data over SPI
                self.bridge.transfer()
            except Exception as ex:
                print("SPI TRANSFER FAILURE")
                print(ex)
            # self.bridge.transfer()
            if self.imuType == IMUType.POLOLU:   
                eulers = self.bridge.get_IMU_angles()
                if eulers is not None:
                    self.eulers = eulers
                acc = self.bridge.get_IMU_accelerometer()
                if acc is not None:
                    self.accelerations = acc
                imuRaw = self.bridge.get_IMU_raw()
                if imuRaw is not None:
                    self.IMURaw = imuRaw
                mag = self.bridge.get_IMU_magnetometer()
                if mag is not None:
                    self.eulerMag = mag
            voltage = self.bridge.get_voltage()
            if voltage is not None:
                self.voltage = voltage
            currAll = self.bridge.get_current_all()
            if currAll is not None:
                self.curAll = currAll
            curLights = self.bridge.get_current_lights()
            if curLights is not None:
                self.curLights = curLights
            for thrusterIndex in range(6):
                for phaseNum, phase in enumerate(('A', 'B', 'C')):
                    current = self.bridge.get_thrusters_phase_current(thrusterIndex, phase)
                    if current is not None:
                        self.thrustersPhaseCurrents[thrusterIndex][phaseNum] = current
            for index in range(3):
                angle = self.bridge.get_man_angles(index)
                if angle is not None:
                    self.manipulator.setAxisTelemetryAngle(index, angle)
                    self.manTelemetryObtained = True
                phaseCurrents = self.bridge.get_man_phase_currents(index)
                if phaseCurrents is not None:
                    self.manipulator.setAxisCurrent(index, phaseCurrents)
                    self.manTelemetryObtained = True
                voltage = self.bridge.get_man_voltage(index)
                if voltage is not None:
                    self.manipulator.setAxisVoltage(index, voltage)
                    self.manTelemetryObtained = True

        self.controlSystem.setAxisValue(Axes.DEPTH, self.depth)
        # print(*["%.2f" % elem for elem in self.eulers], sep ='; ')
        # print(*["%.2f" % elem for elem in self.IMUErrors], sep ='; ')
        self.controlSystem.setAxisValue(Axes.ROLL, self.eulers[0] - self.IMUErrors[0])
        self.controlSystem.setAxisValue(Axes.PITCH, self.eulers[1] - self.IMUErrors[1])
        self.controlSystem.setAxisValue(Axes.YAW, self.eulers[2] - self.IMUErrors[2])
        if self.remoteAddres:
            if not self.newTxPacket:
                telemetry_data = struct.pack('=fffffff', self.controlSystem.getAxisValue(Axes.ROLL), 
                                            self.controlSystem.getAxisValue(Axes.PITCH), 
                                            self.controlSystem.getAxisValue(Axes.YAW), 
                                            map_value(self.cameraAngle, 40, 90, -90, 90),
                                            self.controlSystem.getAxisValue(Axes.DEPTH), 
                                            self.controlSystem.getPIDSetpoint(Axes.ROLL), 
                                            self.controlSystem.getPIDSetpoint(Axes.PITCH))
                
                self.transport.sendto(telemetry_data, self.remoteAddres)
            else:
                self.manTelemetryObtained = True
                if not (self.manTelemetryObtained  and (self.controlType == ControlType.STM_CTRL)):
                    # ERRORFLAGS, roll, pitch, yaw, depth, batVoltage, batCharge, batCurrent, rollSP, pitchSP
                    # mot1PhaseA, mot1PhaseB, mot1PhaseC, mot2PhaseA, mot2PhaseB, mot2PhaseC, mot3PhaseA, mot3PhaseB, mot3PhaseC, 
                    # mot4PhaseA, mot4PhaseB, mot4PhaseC, mot5PhaseA, mot5PhaseB, mot5PhaseC, mot6PhaseA, mot6PhaseB, mot6PhaseC
                    telemetry_data = struct.pack('=Qfffffffffffffffffffffffffff',
                                                self.TelemetryFlags,
                                                self.controlSystem.getAxisValue(Axes.ROLL), 
                                                self.controlSystem.getAxisValue(Axes.PITCH), 
                                                self.controlSystem.getAxisValue(Axes.YAW),
                                                self.controlSystem.getAxisValue(Axes.DEPTH),
                                                self.voltage,
                                                self.batCharge,
                                                self.curAll,
                                                self.controlSystem.getPIDSetpoint(Axes.ROLL), 
                                                self.controlSystem.getPIDSetpoint(Axes.PITCH),
                                                self.thrustersPhaseCurrents[0][0], self.thrustersPhaseCurrents[0][1], self.thrustersPhaseCurrents[0][2],
                                                self.thrustersPhaseCurrents[1][0], self.thrustersPhaseCurrents[1][1], self.thrustersPhaseCurrents[1][2],
                                                self.thrustersPhaseCurrents[2][0], self.thrustersPhaseCurrents[2][1], self.thrustersPhaseCurrents[2][2],
                                                self.thrustersPhaseCurrents[3][0], self.thrustersPhaseCurrents[3][1], self.thrustersPhaseCurrents[3][2],
                                                self.thrustersPhaseCurrents[4][0], self.thrustersPhaseCurrents[4][1], self.thrustersPhaseCurrents[4][2],
                                                self.thrustersPhaseCurrents[5][0], self.thrustersPhaseCurrents[5][1], self.thrustersPhaseCurrents[5][2])
                else:
                    # ERRORFLAGS, roll, pitch, yaw, depth, batVoltage, batCharge, batCurrent, rollSP, pitchSP, 
                    # mot1PhaseA, mot1PhaseB, mot1PhaseC, mot2PhaseA, mot2PhaseB, mot2PhaseC, mot3PhaseA, mot3PhaseB, mot3PhaseC, 
                    # mot4PhaseA, mot4PhaseB, mot4PhaseC, mot5PhaseA, mot5PhaseB, mot5PhaseC, mot6PhaseA, mot6PhaseB, mot6PhaseC,
                    # manAngle1, manAngle2, manAngle3, manPhaseA1, manPhaseB1, manPhaseA2, manPhaseB2, manPhaseA3, manPhaseB3, manVoltage1, manVoltage2, manVoltage3
                    manCurrents = self.manipulator.getCurrents()
                    telemetry_data = struct.pack('=Qfffffffffffffffffffffffffffffffffffffff',
                                                self.TelemetryFlags,
                                                self.controlSystem.getAxisValue(Axes.ROLL), 
                                                self.controlSystem.getAxisValue(Axes.PITCH), 
                                                self.controlSystem.getAxisValue(Axes.YAW),
                                                self.controlSystem.getAxisValue(Axes.DEPTH),
                                                self.voltage,
                                                self.batCharge,
                                                self.curAll,
                                                self.controlSystem.getPIDSetpoint(Axes.ROLL), 
                                                self.controlSystem.getPIDSetpoint(Axes.PITCH),
                                                self.thrustersPhaseCurrents[0][0], self.thrustersPhaseCurrents[0][1], self.thrustersPhaseCurrents[0][2],
                                                self.thrustersPhaseCurrents[1][0], self.thrustersPhaseCurrents[1][1], self.thrustersPhaseCurrents[1][2],
                                                self.thrustersPhaseCurrents[2][0], self.thrustersPhaseCurrents[2][1], self.thrustersPhaseCurrents[2][2],
                                                self.thrustersPhaseCurrents[3][0], self.thrustersPhaseCurrents[3][1], self.thrustersPhaseCurrents[3][2],
                                                self.thrustersPhaseCurrents[4][0], self.thrustersPhaseCurrents[4][1], self.thrustersPhaseCurrents[4][2],
                                                self.thrustersPhaseCurrents[5][0], self.thrustersPhaseCurrents[5][1], self.thrustersPhaseCurrents[5][2],
                                                self.manipulator.getAxisTelemetryAngle(0),
                                                self.manipulator.getAxisTelemetryAngle(1),
                                                self.manipulator.getAxisTelemetryAngle(2),
                                                manCurrents[0][0], manCurrents[0][1],
                                                manCurrents[1][0], manCurrents[1][1],
                                                manCurrents[2][0], manCurrents[2][1],
                                                self.manipulator.getAxisVoltage(0),
                                                self.manipulator.getAxisVoltage(1),
                                                self.manipulator.getAxisVoltage(2))                    
                self.transport.sendto(telemetry_data, self.remoteAddres)

    def shutdown(self):
        self.timer.stop()
        if self.bridge is not None:
            self.bridge.close()
        if self.thrusters is not None:
            self.thrusters.off()
        print("Stop main server")
