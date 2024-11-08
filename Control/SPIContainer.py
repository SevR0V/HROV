import struct
import numpy as np
from enum import IntEnum

# TX_BUFFER: [ MAGIC_START | MOT_SERVO | MAN_Q | ... | MAGIC_END ]
# TX bytes: [MAGIC_START (1x1)(1) | FLAGS (8x1)(8) | LIGHT (2x4)(8) | MOTORS (6x4)(24) | CAM_ANGLE (1x4)(4) | ... | MAGIC_END (1x1)(1) ]
# RX bytes: [MAGIC_START (1x1)(1) | FLAGS (8x1)(8) | EULER (3x4)(12) | EULER_ACC (3x4)(12) | EULER_RAW (3x4)(12) | EULER_MAG (3x4)(12) | CUR_ALL (2x4)(4) | CUR_LIGHT1 (1x4)(4) | CUR_LIGHT2 (1x4)(4) | VOLTS24 (1x4)(4) | ... | MAGIC_END (1x1)(1) ]

class RxBufferOffsets(IntEnum):
    MAGIC_START = 0
    FLAGS = 1
    EULER = 9
    EULER_ACC = 21
    EULER_RAW = 33
    EULER_MAG = 45
    CUR_ALL = 57
    CUR_LIGHT1 = 61
    CUR_LIGHT2 = 65
    VOLTS24 = 69
    MAGIC_END = 199

class TxBufferOffsets(IntEnum):
    MAGIC_START = 0
    FLAGS = 1
    MOTORS = 9
    CAM_ANGLE = 33
    LIGHT = 37 
    MAGIC_END = 199


SPI_RX_EULERx_FLAG =        np.uint64(1 << 0)
SPI_RX_EULER_ACCx_FLAG =    np.uint64(1 << 1)
SPI_RX_EULER_RAWx_FLAG =    np.uint64(1 << 2)
SPI_RX_EULER_MAGx_FLAG =    np.uint64(1 << 3)
SPI_RX_CUR_ALLx_FLAG =      np.uint64(1 << 4)
SPI_RX_CUR_LIGHT1x_FLAG =   np.uint64(1 << 5)
SPI_RX_CUR_LIGHT2x_FLAG =   np.uint64(1 << 6)
SPI_RX_VOLTS24x_FLAG =      np.uint64(1 << 7)

SPI_TX_DES_MOTORSx_FLAG =       np.uint64(1 << 0)
SPI_TX_DES_LIGHTx_FLAG =        np.uint64(1 << 1)
SPI_TX_DES_CAM_ANGLEx_FLAG =    np.uint64(1 << 2)

baseRxPacket = "Qffffffffffffffff"

def countPacketfSize(packetf):
    packetSize = 0
    for char in packetf:
        packetSize += 1 if char == "B" else 0
        packetSize += 4 if char == "f" else 0
        packetSize += 8 if char == "Q" else 0
    return packetSize

def formPacket(defPacket):
    packetSize = countPacketfSize(defPacket)    
    packetSize += 2
    hollowSize = 200 - packetSize
    resPacket = defPacket
    for i in range(hollowSize):
        resPacket += "B"
    resPacket = "B" + resPacket + "B"
    return resPacket


class SPI_Xfer_Container:
    BUFFER_SIZE = 200
    MAGIC_START = 0xAB
    MAGIC_END = 0xCD

    def __init__(self, pi, spi_channel, spi_speed, spi_flags):
        self.pi = pi
        self.spi_handle = self.pi.spi_open(spi_channel, spi_speed, spi_flags)

        self.rx_buffer = bytearray(self.BUFFER_SIZE)
        self.tx_buffer = bytearray(self.BUFFER_SIZE)

        # TX
        self.des_lights = [0.0, 0.0,]
        self.des_mot = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,]
        self.des_cam_angle = 0.0

        # RX
        self.euler =    [0.0, 0.0, 0.0]
        self.eulerAcc = [0.0, 0.0, 0.0]
        self.eulerRaw = [0.0, 0.0, 0.0]
        self.eulerMag = [0.0, 0.0, 0.0]
        self.currentAll = 0
        self.cirrentLight1 = 0
        self.cirrentLight2 = 0
        self.voltage24 = 0

        self.tx_refresh_flags = np.uint64(0)
        self.rx_refresh_flags = np.uint64(0)

    def _parse_rx_buffer(self):
        if (self.rx_buffer[RxBufferOffsets.MAGIC_START] != self.MAGIC_START or
            self.rx_buffer[RxBufferOffsets.MAGIC_END] != self.MAGIC_END):
            return False
        rxPacket = 0
        try:
            rxPacket = struct.unpack_from("=" + formPacket(baseRxPacket), self.rx_buffer)
        except:
            print("SPI RX WRONG PACKET")
            return False
        self.rx_refresh_flags = np.uint64(rxPacket[1])
        self.euler = [rxPacket[2], rxPacket[3], rxPacket[4]] if self.rx_refresh_flags & SPI_RX_EULERx_FLAG else None
        self.eulerAcc = [rxPacket[5], rxPacket[6], rxPacket[7]] if self.rx_refresh_flags & SPI_RX_EULER_ACCx_FLAG else None
        self.eulerRaw = [rxPacket[8], rxPacket[9], rxPacket[10]] if self.rx_refresh_flags & SPI_RX_EULER_RAWx_FLAG else None
        self.eulerMag = [rxPacket[11], rxPacket[12], rxPacket[13]] if self.rx_refresh_flags & SPI_RX_EULER_MAGx_FLAG else None
        self.currentAll = rxPacket[14] if self.rx_refresh_flags & SPI_RX_CUR_ALLx_FLAG else None
        self.cirrentLight1 = rxPacket[15] if self.rx_refresh_flags & SPI_RX_CUR_LIGHT1x_FLAG else None
        self.cirrentLight2 = rxPacket[16] if self.rx_refresh_flags & SPI_RX_CUR_LIGHT2x_FLAG else None
        self.voltage24 = rxPacket[17] if self.rx_refresh_flags & SPI_RX_VOLTS24x_FLAG else None
        return True

    def _fill_tx_buffer(self):
        self.tx_buffer[TxBufferOffsets.MAGIC_START] = self.MAGIC_START
        self.tx_buffer[TxBufferOffsets.MAGIC_END] = self.MAGIC_END

        struct.pack_into('Q', self.tx_buffer, TxBufferOffsets.FLAGS, self.tx_refresh_flags)
                
        if self.tx_refresh_flags & SPI_TX_DES_LIGHTx_FLAG:
            for index in range(0, 2):           
                offset = TxBufferOffsets.LIGHT + 4 * index
                struct.pack_into('f', self.tx_buffer, offset, self.des_lights[index])

        if self.tx_refresh_flags & SPI_TX_DES_MOTORSx_FLAG:
            for index in range(0, 6):           
                offset = TxBufferOffsets.MOTORS + 4 * index
                struct.pack_into('f', self.tx_buffer, offset, self.des_mot[index])
        
        if self.tx_refresh_flags & SPI_TX_DES_CAM_ANGLEx_FLAG:
            offset = TxBufferOffsets.CAM_ANGLE
            struct.pack_into('f', self.tx_buffer, offset, self.des_cam_angle)
        #self.tx_refresh_flags = np.uint64(0)

    def transfer(self):
        self._fill_tx_buffer()
        count, self.rx_buffer = self.pi.spi_xfer(self.spi_handle, self.tx_buffer)
        return self._parse_rx_buffer()

    # Setters
    def set_mots_values(self, values):
        self.des_mot = values
        self.tx_refresh_flags |= SPI_TX_DES_MOTORSx_FLAG
    
    def set_lights_values(self, values):
        self.des_lights = values
        self.tx_refresh_flags |= SPI_TX_DES_LIGHTx_FLAG
        
    def set_cam_angle_value(self, value):
        self.des_cam_angle = value
        self.tx_refresh_flags |= SPI_TX_DES_CAM_ANGLEx_FLAG

    # Getters
    def get_IMU_angles(self):
        if self.rx_refresh_flags & SPI_RX_EULERx_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_EULERx_FLAG)
            return self.euler
        return None
    
    def get_IMU_raw(self):
        if self.rx_refresh_flags & SPI_RX_EULER_RAWx_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_EULER_RAWx_FLAG)
            return self.eulerRaw
        return None
    
    def get_IMU_accelerometer(self):
        if self.rx_refresh_flags & SPI_RX_EULER_ACCx_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_EULER_ACCx_FLAG)
            return self.eulerAcc
        return None
    
    def get_IMU_magnetometer(self):
        if self.rx_refresh_flags & SPI_RX_EULER_MAGx_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_EULER_MAGx_FLAG)
            return self.eulerMag
        return None
    
    def get_current_all(self):
        if self.rx_refresh_flags & SPI_RX_CUR_ALLx_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_CUR_ALLx_FLAG)
            return self.currentAll
        return None
    
    def get_current_lights(self):
        light1 = None
        light2 = None
        if self.rx_refresh_flags & SPI_RX_CUR_LIGHT1x_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_CUR_LIGHT1x_FLAG)
            light1 =  self.cirrentLight1
            
        if self.rx_refresh_flags & SPI_RX_CUR_LIGHT2x_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_CUR_LIGHT2x_FLAG)
            light2 =  self.cirrentLight2
        
        if light1 is not None or light2 is not None:
            return [light1, light2]            
        return None
    
    def get_voltage(self):
        if self.rx_refresh_flags & SPI_RX_VOLTS24x_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_VOLTS24x_FLAG)
            return self.voltage24
        return None

    def close(self):
        self.pi.spi_close(self.spi_handle)
        self.pi.stop()
