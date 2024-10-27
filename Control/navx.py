import asyncio
import re
import time


class Navx(asyncio.Protocol):
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.heading = 0
        self.pattern = re.compile(r'-?\d{3}\.\d{2}')
        self.transport = None
        self.on_data = None
        self.angular_velocity = 0
        self.elapsed_time = 0
        self.last_time = time.process_time()

    def subscribe(self, callback):
        self.on_data = callback

    def connection_made(self, transport):
        transport.serial.rts = False
        self.transport = transport
        command = b"!Sy3252\r\n"
        transport.write(command)

    def data_received(self, data):
        data = str(data[2:-4])
        # print(data)

        matched = self.pattern.findall(data)
        if matched and len(matched) == 4:
            now = time.process_time()
            self.elapsed_time = now - self.last_time
            self.last_time = now

            matched = [float(value) for value in matched]
            last_yaw = self.yaw
            self.yaw, self.pitch, self.roll, self.heading = matched
            self.angular_velocity = (self.yaw - last_yaw) * self.elapsed_time

            self.on_data(self, [self.roll, self.pitch, self.yaw, self.heading])


    def connection_lost(self, exc):
        asyncio.get_event_loop().stop()
