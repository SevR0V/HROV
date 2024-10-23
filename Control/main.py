import asyncio
import time
import pigpio
from remoteserver import RemoteUdpDataServer
from SPIContainer import SPI_Xfer_Container
from asynctimer import AsyncTimer
from HROVcontrolsystem import HROVControlSystem

time.sleep(2)

pi = pigpio.pi()
loop = asyncio.get_event_loop()

#init SPI parameters
SPIChannel = 0
SPISpeed = 500000
SPIFlags = 0

#init thrusters parameters
thrustersDirCorr = [1, 1, 1, 1, 1, 1]
thrustersOrder = [HROVControlSystem.Thrusters.H_FORW_TOP, 
                  HROVControlSystem.Thrusters.H_FORW_BOT,
                  HROVControlSystem.Thrusters.H_SIDE_FRONT, 
                  HROVControlSystem.Thrusters.H_SIDE_REAR,
                  HROVControlSystem.Thrusters.V_RIGHT,
                  HROVControlSystem.Thrusters.V_LEFT]
trustersXValues = [-500, 500]

#init control system
controlSystem = HROVControlSystem()
controlSystem.set_thrusters_calibration_values(thrustersDirCorr, thrustersOrder, trustersXValues, 1.5)

#init timer parameters
timerInterval = 1/500 #500 Hz timer interval

#init devices
timer = timer = AsyncTimer(timerInterval, loop)
bridge = SPI_Xfer_Container(pi, SPIChannel, SPISpeed, SPIFlags)

#init main server
udp_server = RemoteUdpDataServer(controlSystem, timer, bridge)

#create tasks
udp_server_task = loop.create_datagram_endpoint(lambda: udp_server, local_addr=('0.0.0.0', 1337))

#register tasks
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
