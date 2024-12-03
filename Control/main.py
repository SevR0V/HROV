import asyncio
import pigpio
from remoteserver import RemoteUdpDataServer
from remoteserver import IMUType
from remoteserver import ControlType
from SPIContainer import SPI_Xfer_Container
from asynctimer import AsyncTimer
from HROVcontrolsystem import ControlSystem
from HROVcontrolsystem import ThrustersNames
from navx import Navx
import serial_asyncio
from thruster import Thrusters
from ligths import Lights
from servo import Servo
from async_hiwonder_reader import AsyndHiwonderReader
from MultiaxisManipulator import MultiaxisManipulator

#select IMU
# IMUType.NAVX
# IMUType.POLOLU
# IMUType.HIWONDER
imuType = IMUType.HIWONDER

#select contol type
# ControlType.DIRECT_CTRL
# ControlType.STM_CTRL
controlType = ControlType.STM_CTRL

if controlType == ControlType.DIRECT_CTRL and imuType == IMUType.POLOLU:
    print("Wrong IMU Type")
    exit()
    imuType = IMUType.NAVX

pi = pigpio.pi()
loop = asyncio.get_event_loop()

bridge = None
navx = None
lights = None
thrusters = None
cameraServo = None
udp_server = None
hiwonderIMU = None
hiwonderTimer = None
hiwonderReader = None
manipulator = MultiaxisManipulator(3)

#init thrusters parameters
thrustersOrder = [ThrustersNames.H_FORW_BOT,
                  ThrustersNames.H_FORW_TOP, 
                  ThrustersNames.V_RIGHT, 
                  ThrustersNames.V_LEFT,
                  ThrustersNames.H_SIDE_REAR,
                  ThrustersNames.H_SIDE_FRONT]
#thrustersDirCorr = [1, -1, 1, 1, -1, 1]
thrustersDirCorr = [-1, 1, -1, 1, 1, 1]
trustersXValues = [-100, 100]

#init control system
controlSystem = ControlSystem()
controlSystem.setThrustersCalibrationValues(thrustersDirCorr, thrustersOrder, trustersXValues, 2)

#init timer parameters
timerInterval = 1/200 #300 Hz timer interval

#init timer
timer = AsyncTimer(timerInterval, loop)

if imuType == IMUType.NAVX:
    #init NavX
    navx = Navx()

if imuType == IMUType.HIWONDER:
    hiwonderReader = AsyndHiwonderReader(1/200, loop,'/dev/ttyUSB0', 38400)

if controlType == ControlType.STM_CTRL:
    #init SPI parameters
    SPIChannel = 0
    SPISpeed = 300000
    SPIFlags = 0
    bridge = SPI_Xfer_Container(pi, SPIChannel, SPISpeed, SPIFlags)

    #init main server
    udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType, manipulator, bridge, navx, hiwonderReader= hiwonderReader)

if controlType == ControlType.DIRECT_CTRL:
    #init thrusters
    thrustersPins = [0] * 6

    thrustersPins[thrustersOrder.index(ThrustersNames.H_FORW_TOP)]     = 22
    thrustersPins[thrustersOrder.index(ThrustersNames.V_RIGHT)]    = 9
    thrustersPins[thrustersOrder.index(ThrustersNames.H_FORW_BOT)]           = 27
    thrustersPins[thrustersOrder.index(ThrustersNames.H_SIDE_REAR)]     = 10
    thrustersPins[thrustersOrder.index(ThrustersNames.H_SIDE_FRONT)]    = 11
    thrustersPins[thrustersOrder.index(ThrustersNames.V_LEFT)]           = 17

    thrusters = Thrusters(pi, thrustersPins, [16], 
                          [[20, 20],[20, 20],[20, 20],[20, 20],[20, 20],[20, 20]],
                          #[[0, 0],[0, 0],[0, 0],[0, 0],[0, 0],[0, 0]],
                          [[0, 0],[0, 0],[0, 0],[0, 0],[0, 0],[0, 0]])

    #init servos
    cameraServo = Servo(pi, 5, [0, 90])

    #init lights
    lights = Lights(pi, [19, 26])

    #init main server
    if imuType == IMUType.NAVX:
        udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType, manipulator, bridge, navx, thrusters, lights, cameraServo)
    if imuType == IMUType.HIWONDER:
        udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType, manipulator, bridge, navx, thrusters, 
                                         lights, cameraServo, hiwonderReader)

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
except Exception as ex:
    print(ex)
