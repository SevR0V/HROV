import asyncio
from typing import Callable, List
from hiwonderIMU import HiwonderIMU

class AsyndHiwonderReader:
    def __init__(self, interval: float, loop: asyncio.AbstractEventLoop, port, baudRate):

        self.hiwonderIMU = HiwonderIMU(port, baudRate)
        self.interval = interval
        self.loop = loop
        self._task = None
        self._running = False
        self.IMUData = [0.0]*9
        self.angles = [0.0]*3

    async def _run(self):
        while self._running:
            await asyncio.sleep(self.interval)
            imuOtput = self.hiwonderIMU.readIMU()
            if imuOtput:
                self.IMUData = imuOtput
                for i in range(3):
                    self.angles[i] = imuOtput[i+6]

    def getInterval(self):
        return self.interval
    
    def getIMUReadings(self):
        return self.IMUData
    
    def getIMUAngles(self):
        return self.angles

    def start(self) -> None:
        if not self._running:
            self._running = True
            self._task = self.loop.create_task(self._run())

    def stop(self) -> None:
        if self._running:
            self._running = False
            if self._task:
                self._task.cancel()
                self._task = None