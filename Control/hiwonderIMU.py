# coding:UTF-8
# Version: V1.0.1
import serial

a = [0.0]*3
w = [0.0]*3
Angle = [0.0]*3

class HiwonderIMU:
    def __init__(self, port, baudRate):
        self.serialConnection = serial.Serial(port, baudRate, timeout=0)
        self.ACCData = [0.0]*8
        self.GYROData = [0.0]*8
        self.AngleData = [0.0]*8
        self.FrameState = 0  # What is the state of the judgment
        self.Bytenum = 0  # Read the number of digits in this paragraph
        self.CheckSum = 0  # Sum check bit 
        self.acc = None
        self.gyro = None
        self.Angle = None

    def readIMU(self):
        datahex = self.serialConnection.read(33)
        data = self.DueData(datahex)
        return data

    def DueData(self, inputdata):  # New core procedures, read the data partition, each read to the corresponding array 
        for data in inputdata:  # Traversal the input data
            if self.FrameState == 0:  # When the state is not determined, enter the following judgment
                if data == 0x55 and self.Bytenum == 0:  # When 0x55 is the first digit, start reading data and increment self.Bytenum
                    self.CheckSum = data
                    self.Bytenum = 1
                    continue
                elif data == 0x51 and self.Bytenum == 1:  # Change the frame if byte is not 0 and 0x51 is identified
                    self.CheckSum += data
                    self.FrameState = 1
                    self.Bytenum = 2
                elif data == 0x52 and self.Bytenum == 1:
                    self.CheckSum += data
                    self.FrameState = 2
                    self.Bytenum = 2
                elif data == 0x53 and self.Bytenum == 1:
                    self.CheckSum += data
                    self.FrameState = 3
                    self.Bytenum = 2
            elif self.FrameState == 1:  # acc

                if self.Bytenum < 10:            # Read 8 data
                    self.ACCData[self.Bytenum-2] = data  # Starting from 0
                    self.CheckSum += data
                    self.Bytenum += 1
                else:
                    if data == (self.CheckSum & 0xff):  # verify check bit
                        self.acc = self.get_acc(self.ACCData)
                    self.CheckSum = 0  # Each data is zeroed and a new circular judgment is made
                    self.Bytenum = 0
                    self.FrameState = 0
            elif self.FrameState == 2:  # gyro

                if self.Bytenum < 10:
                    self.GYROData[self.Bytenum-2] = data
                    self.CheckSum += data
                    self.Bytenum += 1
                else:
                    if data == (self.CheckSum & 0xff):
                        self.gyro = self.get_gyro(self.GYROData)
                    self.CheckSum = 0
                    self.Bytenum = 0
                    self.FrameState = 0
            elif self.FrameState == 3:  # angle

                if self.Bytenum < 10:
                    self.AngleData[self.Bytenum-2] = data
                    self.CheckSum += data
                    self.Bytenum += 1
                else:
                    if data == (self.CheckSum & 0xff):
                        self.Angle = self.get_angle(self.AngleData)
                        result = 0
                        if self.acc and self.gyro and self.Angle:
                            result = self.acc+self.gyro+self.Angle
                        self.CheckSum = 0
                        self.Bytenum = 0
                        self.FrameState = 0
                        return result
                    self.CheckSum = 0
                    self.Bytenum = 0
                    self.FrameState = 0
                    return 0


    def get_acc(self, datahex):
        axl = datahex[0]
        axh = datahex[1]
        ayl = datahex[2]
        ayh = datahex[3]
        azl = datahex[4]
        azh = datahex[5]
        k_acc = 16.0
        acc_x = (axh << 8 | axl) / 32768.0 * k_acc
        acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
        acc_z = (azh << 8 | azl) / 32768.0 * k_acc
        if acc_x >= k_acc:
            acc_x -= 2 * k_acc
        if acc_y >= k_acc:
            acc_y -= 2 * k_acc
        if acc_z >= k_acc:
            acc_z -= 2 * k_acc
        return acc_x, acc_y, acc_z


    def get_gyro(self, datahex):
        wxl = datahex[0]
        wxh = datahex[1]
        wyl = datahex[2]
        wyh = datahex[3]
        wzl = datahex[4]
        wzh = datahex[5]
        k_gyro = 2000.0
        gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
        gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
        gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
        if gyro_x >= k_gyro:
            gyro_x -= 2 * k_gyro
        if gyro_y >= k_gyro:
            gyro_y -= 2 * k_gyro
        if gyro_z >= k_gyro:
            gyro_z -= 2 * k_gyro
        return gyro_x, gyro_y, gyro_z


    def get_angle(self, datahex):
        rxl = datahex[0]
        rxh = datahex[1]
        ryl = datahex[2]
        ryh = datahex[3]
        rzl = datahex[4]
        rzh = datahex[5]
        k_angle = 180.0
        angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
        angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
        angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
        if angle_x >= k_angle:
            angle_x -= 2 * k_angle
        if angle_y >= k_angle:
            angle_y -= 2 * k_angle
        if angle_z >= k_angle:
            angle_z -= 2 * k_angle
        return angle_x, angle_y, angle_z


if __name__ == '__main__':
    port = '/dev/ttyUSB1' # USB serial port 
    baud = 9600   # Same baud rate as the INERTIAL navigation module
    imu = HiwonderIMU(port, baud)
    while(1):
        imuOtput = imu.readIMU()
        if imuOtput:
            accel = [0]*3
            gyro = [0]*3
            angles = [0]*3
            for i in range(3):
                angles[i] = imuOtput[i+6]
            #print("acc:%10.3f %10.3f %10.3f gyro:%10.3f %10.3f %10.3f angle:%10.3f %10.3f %10.3f" % imuOtput)
            print(["%.2f" % elem for elem in angles], sep ='; ')
