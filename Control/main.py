import asyncio
import pigpio
from remoteserver import RemoteUdpDataServer
from remoteserver import IMUType
from remoteserver import ControlType
from SPIContainer import SPI_Xfer_Container
from asynctimer import AsyncTimer
from HROVcontrolsystem import ControlSystem
from navx import Navx
import serial_asyncio
from thruster import Thrusters
from ligths import Lights
from servo import Servo

#select IMU
# IMUType.NAVX
# IMUType.POLOLU
imuType = IMUType.POLOLU

#select contol type
# ControlType.DIRECT_CTRL
# ControlType.STM_CTRL
controlType = ControlType.STM_CTRL

if controlType == ControlType.DIRECT_CTRL:
    imuType = IMUType.NAVX

pi = pigpio.pi()
loop = asyncio.get_event_loop()

bridge = None
navx = None
lights = None
thrusters = None
cameraServo = None
udp_server = None

#init thrusters parameters
thrustersOrder = [ControlSystem.Thrusters.H_FORW_TOP, 
                  ControlSystem.Thrusters.H_FORW_BOT,
                  ControlSystem.Thrusters.H_SIDE_FRONT, 
                  ControlSystem.Thrusters.H_SIDE_REAR,
                  ControlSystem.Thrusters.V_RIGHT,
                  ControlSystem.Thrusters.V_LEFT]
thrustersDirCorr = [1, 1, 1, 1, 1, 1]
trustersXValues = [-50, 50]

#init control system
controlSystem = ControlSystem()
controlSystem.setThrustersCalibrationValues(thrustersDirCorr, thrustersOrder, trustersXValues, 2)

#init timer parameters
timerInterval = 1/300 #300 Hz timer interval

#init timer
timer = timer = AsyncTimer(timerInterval, loop)

if imuType == IMUType.NAVX:
    #init NavX
    navx = Navx()

if controlType == ControlType.STM_CTRL:
    #init SPI parameters
    SPIChannel = 0
    SPISpeed = 500000
    SPIFlags = 0
    bridge = SPI_Xfer_Container(pi, SPIChannel, SPISpeed, SPIFlags)

    #init main server
    udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType, bridge, navx)

if controlType == ControlType.DIRECT_CTRL:
    #init thrusters
    thrustersPins = [0] * 6

    thrustersPins[thrustersOrder.index(ControlSystem.Thrusters.H_FORW_TOP)]    = 10
    thrustersPins[thrustersOrder.index(ControlSystem.Thrusters.H_FORW_BOT)]    = 9
    thrustersPins[thrustersOrder.index(ControlSystem.Thrusters.H_SIDE_FRONT)]  = 17
    thrustersPins[thrustersOrder.index(ControlSystem.Thrusters.H_SIDE_REAR)]   = 22
    thrustersPins[thrustersOrder.index(ControlSystem.Thrusters.V_RIGHT)]       = 27
    thrustersPins[thrustersOrder.index(ControlSystem.Thrusters.V_LEFT)]        = 11

    thrusters = Thrusters(pi, thrustersPins, [16], [[0, 0],[0, 0],[0, 0],[0, 0],[0, 0],[0, 0]], [[0, 0],[0, 0],[0, 0],[0, 0],[0, 0],[0, 0]])

    #init servos
    cameraServo = Servo(pi, 5, [0, 90])

    #init lights
    lights = Lights(pi, [19, 26])

    #init main server
    udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType, bridge, navx, thrusters, lights, cameraServo)

#create tasks
serial_task = None
task = None
udp_server_task = loop.create_datagram_endpoint(lambda: udp_server, local_addr=('0.0.0.0', 1337))
if imuType == IMUType.NAVX:
    serial_task = serial_asyncio.create_serial_connection(loop, lambda: navx, '/dev/ttyACM0', baudrate=115200)

#register tasks
if imuType == IMUType.NAVX:
    task = asyncio.gather(udp_server_task, serial_task, return_exceptions=True)
else:
    task = asyncio.gather(udp_server_task, return_exceptions=True)

#start main loop
loop.run_until_complete(task)
try:
    loop.run_forever()
except KeyboardInterrupt:
    udp_server.shutdown()
    loop.close()
    pi.stop()
    print("Shutdown")
