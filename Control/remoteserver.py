import asyncio
import struct
import math
import time
import ms5837
import numpy as np
from enum import IntEnum
from SPIContainer import SPI_Xfer_Container
from HROVcontrolsystem import HROVControlSystem
from asynctimer import AsyncTimer
from utils import map_value

to_rad = math.pi / 180

UDP_FLAGS_MASTERx = np.uint64(1 << 0)
UDP_FLAGS_LIGHT_STATEx = np.uint64(1 << 1)
UDP_FLAGS_STAB_ROLLx = np.uint64(1 << 2)
UDP_FLAGS_STAB_PITCHx = np.uint64(1 << 3)
UDP_FLAGS_STAB_YAWx = np.uint64(1 << 4)
UDP_FLAGS_STAB_DEPTHx = np.uint64(1 << 5)
UDP_FLAGS_RESET_POSITIONx = np.uint64(1 << 6)
UDP_FLAGS_RESET_IMUx = np.uint64(1 << 7)
UDP_FLAGS_UPDATE_PIDx = np.uint64(1 << 8)

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

class RemoteUdpDataServer(asyncio.Protocol):
    def __init__(self, contolSystem: HROVControlSystem, timer: AsyncTimer, bridge: SPI_Xfer_Container):
        self.timer = timer
        self.bridge = bridge
        self.remoteAddres = None
        timer.subscribe(self.dataCalculationTransfer)
        timer.start()
        self.controlSystem = contolSystem
        self.powerTarget = 0
        self.cameraRotate = 0
        self.cameraAngle = 0
        self.lightState = 0
        self.eulers = [0.0, 0.0, 0.0]
        self.accelerations = [0.0, 0.0, 0.0]
        self.eulerMag = [0.0, 0.0, 0.0]
        self.voltage = 0
        self.curAll = 0
        self.curLights = [0.0, 0.0]
        self.depth = 0
        self.MASTER = True
        self.IMUErrors = [0.0, 0.0, 0.0]
        self.incrementScale = 0.5
        self.batCharge = 0

        self.ERRORFLAGS = np.uint64(0)
        
        self.newRxPacket = False
        self.newTxPacket = False
        
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
            
            self.controlSystem.setAxisInput(self.controlSystem.ControlAxes.STRAFE, received[0] * 100)
            self.controlSystem.setAxisInput(self.controlSystem.ControlAxes.FORWARD, received[1] * 100)
            self.controlSystem.setAxisInput(self.controlSystem.ControlAxes.DEPTH, received[2] * 100)
            self.controlSystem.setAxisInput(self.controlSystem.ControlAxes.YAW, received[4] * 100) 

            self.powerTarget = received[3]
            self.cameraRotate = received[7]
            self.cameraAngle += self.cameraRotate * self.incrementScale
            self.lightState = received[9]

            rollStab =  1 if received[10] & 0b00000001 else 0
            pitchStab = 1 if received[10] & 0b00000010 else 0
            yawStab =   1 if received[10] & 0b00000100 else 0
            depthStab = 1 if received[10] & 0b00001000 else 0

            self.controlSystem.setStabilization(self.controlSystem.ControlAxes.ROLL, rollStab)
            self.controlSystem.setStabilization(self.controlSystem.ControlAxes.PITCH, pitchStab)
            self.controlSystem.setStabilization(self.controlSystem.ControlAxes.YAW, yawStab)
            self.controlSystem.setStabilization(self.controlSystem.ControlAxes.DEPTH, depthStab)
            
            if received[11]: 
                rollSP = self.controlSystem.getPIDSetpoint(self.controlSystem.ControlAxes.ROLL) + received[11] * self.incrementScale
                self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.ROLL, rollSP)
            if received[12]: 
                pitchSP = self.controlSystem.getPIDSetpoint(self.controlSystem.ControlAxes.PITCH) + received[12] * self.incrementScale
                self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.PITCH, pitchSP)
            
            if(received[13]):
                self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.ROLL, 0)
                self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.PITCH, 0)
                self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.YAW, self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.YAW))
                self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.DEPTH, self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.DEPTH))
                self.controlSystem.setStabilizations([0,0,0,0,0,0])
                
            if(received[26]):
                    self.controlSystem.setPIDConstants(self.controlSystem.ControlAxes.ROLL, [received[14], received[15], received[16]])
                    self.controlSystem.setPIDConstants(self.controlSystem.ControlAxes.PITCH, [received[17], received[18], received[19]])
                    self.controlSystem.setPIDConstants(self.controlSystem.ControlAxes.YAW, [received[20], received[21], received[22]])
                    self.controlSystem.setPIDConstants(self.controlSystem.ControlAxes.DEPTH, [received[23], received[24], received[25]])
        else:
            # controlFlags, forward, strafe, vertical, rotation, rollInc, pitchInc, powerTarget, cameraRotate, manipulatorGrip, manipulatorRotate, rollKp, rollKi, rollKd, pitchKp, pitchKi, pitchKd, yawKp, yawKi, yawKd, depthKp, depthKi, depthKd
            # flags = MASTER, lightState, stabRoll, stabPitch, stabYaw, stabDepth, resetPosition, resetIMU, updatePID
            received = struct.unpack_from("=Qfffffffffffffffffff", packet)
            self.MASTER = received[UDPRxValues.FLAGS] & UDP_FLAGS_MASTERx
            self.lightState = received[UDPRxValues.FLAGS] & UDP_FLAGS_LIGHT_STATEx
            rollStab =  received[UDPRxValues.FLAGS] & UDP_FLAGS_STAB_ROLLx
            pitchStab = received[UDPRxValues.FLAGS] & UDP_FLAGS_STAB_PITCHx
            yawStab =   received[UDPRxValues.FLAGS] & UDP_FLAGS_STAB_YAWx
            depthStab = received[UDPRxValues.FLAGS] & UDP_FLAGS_STAB_DEPTHx
            resetPosition = received[UDPRxValues.FLAGS] & UDP_FLAGS_RESET_POSITIONx
            resetIMU = received[UDPRxValues.FLAGS] & UDP_FLAGS_RESET_IMUx
            updatePID = received[UDPRxValues.FLAGS] & UDP_FLAGS_UPDATE_PIDx
            
            self.controlSystem.setStabilization(self.controlSystem.ControlAxes.ROLL, rollStab)
            self.controlSystem.setStabilization(self.controlSystem.ControlAxes.PITCH, pitchStab)
            self.controlSystem.setStabilization(self.controlSystem.ControlAxes.YAW, yawStab)
            self.controlSystem.setStabilization(self.controlSystem.ControlAxes.DEPTH, depthStab) 
            
            self.controlSystem.setAxisInput(self.controlSystem.ControlAxes.FORWARD, received[UDPRxValues.FORWARD] * 100)
            self.controlSystem.setAxisInput(self.controlSystem.ControlAxes.STRAFE, received[UDPRxValues.STRAFE] * 100)
            self.controlSystem.setAxisInput(self.controlSystem.ControlAxes.DEPTH, received[UDPRxValues.VERTICAL] * 100)
            self.controlSystem.setAxisInput(self.controlSystem.ControlAxes.YAW, received[UDPRxValues.ROTATION] * 100) 
            rollInc = received[UDPRxValues.ROLL_INC]
            pitchInc = received[UDPRxValues.ROLL_INC]
            if rollInc:
                rollSP = self.controlSystem.getPIDSetpoint(self.controlSystem.ControlAxes.ROLL) + rollInc * self.incrementScale
                self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.ROLL, rollSP)
            if pitchInc:
                pitchSP = self.controlSystem.getPIDSetpoint(self.controlSystem.ControlAxes.PITCH) + pitchInc * self.incrementScale
                self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.PITCH, pitchSP)
            self.powerTarget = received[UDPRxValues.POWER_TARGET]
            self.cameraRotate = received[UDPRxValues.CAM_ROTATE]            
            self.cameraAngle += self.cameraRotate * self.incrementScale
            
            if updatePID:
                self.controlSystem.setPIDConstants(self.controlSystem.ControlAxes.ROLL, [received[UDPRxValues.ROLL_KP], received[UDPRxValues.ROLL_KI], received[UDPRxValues.ROLL_KD]])
                self.controlSystem.setPIDConstants(self.controlSystem.ControlAxes.PITCH, [received[UDPRxValues.PITCH_KP], received[UDPRxValues.PITCH_KI], received[UDPRxValues.PITCH_KD]])
                self.controlSystem.setPIDConstants(self.controlSystem.ControlAxes.YAW, [received[UDPRxValues.YAW_KP], received[UDPRxValues.YAW_KI], received[UDPRxValues.YAW_KD]])
                self.controlSystem.setPIDConstants(self.controlSystem.ControlAxes.DEPTH, [received[UDPRxValues.DEPTH_KP], received[UDPRxValues.DEPTH_KI], received[UDPRxValues.DEPTH_KD]])
            
            if resetPosition:
                self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.ROLL, 0)
                self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.PITCH, 0)
                self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.YAW, self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.YAW))
                self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.DEPTH, self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.DEPTH))
                self.controlSystem.setStabilizations([0,0,0,0,0,0])
            
            if resetIMU:
                self.IMUErrors = [self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.ROLL),
                                  self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.PITCH),
                                  self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.YAW)]
                       
        self.controlSystem.setdt(self.timer.getInterval())

    def dataCalculationTransfer(self):
        if self.ds_init:
            if self.depth_sensor.read(ms5837.OSR_256):
                self.depth = self.depth_sensor.pressure(ms5837.UNITS_atm)*10-10
        if self.MASTER:
            self.bridge.set_cam_angle_value(self.cameraAngle)
            lightsValues = [50*self.lightState, 50*self.lightState]
            self.bridge.set_lights_values(lightsValues)
            self.bridge.set_mots_values(self.controlSystem.getMotsControls())
            self.bridge.set_cam_angle_value(self.cameraAngle)     
        try:
            # Transfer data over SPI
            self.bridge.transfer()
        except:
            print("SPI TRANSFER FAILURE")        
        eulers = self.bridge.get_IMU_angles()
        if eulers is not None:
            self.eulers = eulers 
        else:
            print("Gyro data read error")           
        voltage = self.bridge.get_voltage()
        if voltage is not None:
            self.voltage = voltage
        else:
            print("Voltage read error") 
        currAll = self.bridge.get_current_all()
        if currAll is not None:
            self.curAll = currAll
        else:
            print("Current read error") 
        curLights = self.bridge.get_current_lights()
        if curLights is not None:
            self.curLights = curLights
        else:
            print("Lights current read error")
        acc = self.bridge.get_IMU_accelerometer()
        if acc is not None:
            self.accelerations = acc
        else:
            print("Accelerations read error")
        mag = self.bridge.get_IMU_magnetometer()
        if mag is not None:
            self.eulerMag = mag
        else:
            print("Magnetometer date read error")
        self.controlSystem.setAxesValues([0, 0, 
                                          self.depth, 
                                          self.eulers[0] - self.IMUErrors[0], 
                                          self.eulers[1] - self.IMUErrors[1], 
                                          self.eulers[2] - self.IMUErrors[2]])

        if self.remoteAddres:
            if not self.newTxPacket:
                telemetry_data = struct.pack('=fffffff', self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.ROLL), 
                                            self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.PITCH), 
                                            self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.YAW), 
                                            0.0, 
                                            self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.DEPTH), 
                                            self.controlSystem.getPIDSetpoint(self.controlSystem.ControlAxes.ROLL), 
                                            self.controlSystem.getPIDSetpoint(self.controlSystem.ControlAxes.PITCH))
                
                self.transport.sendto(telemetry_data, self.remoteAddres)
            else:
                #ERRORFLAGS, roll, pitch, yaw, depth, batVoltage, batCharge, batCurrent, rollSP, pitchSP
                telemetry_data = struct.pack('=Qfffffffff',
                                            self.ERRORFLAGS,
                                            self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.ROLL), 
                                            self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.PITCH), 
                                            self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.YAW),
                                            self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.DEPTH),
                                            self.voltage,
                                            self.batCharge,
                                            self.curAll,
                                            self.controlSystem.getPIDSetpoint(self.controlSystem.ControlAxes.ROLL), 
                                            self.controlSystem.getPIDSetpoint(self.controlSystem.ControlAxes.PITCH))
                
                self.transport.sendto(telemetry_data, self.remoteAddres)
                

    def shutdown(self):       
        self.bridge.close()  
        print("Stop main server")
