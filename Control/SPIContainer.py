import struct
import numpy as np
from enum import IntEnum
from MultiaxisManipulator import GripState
import copy

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
    MOT1_CURRENTS = 73
    MOT2_CURRENTS = 85
    MOT3_CURRENTS = 97
    MOT4_CURRENTS = 109
    MOT5_CURRENTS = 121
    MOT6_CURRENTS = 133
    MAN1_VOLTAGE = 145
    MAN1_PHASES = 149
    MAN1_ANGLE = 157
    MAN2_VOLTAGE = 161
    MAN2_PHASES = 165
    MAN2_ANGLE = 173
    MAN3_VOLTAGE = 177
    MAN3_PHASES = 181
    MAN3_ANGLE = 193
    MAGIC_END = 199

class TxBufferOffsets(IntEnum):
    MAGIC_START = 0
    FLAGS = 1
    MOTORS = 9
    CAM_ANGLE = 33
    LIGHT = 37
    MAN_ANGLES = 45
    MAN_GRIP = 57
    MAGIC_END = 199


SPI_RX_EULER_FLAG =        np.uint64(1 << 0)
SPI_RX_EULER_ACC_FLAG =    np.uint64(1 << 1)
SPI_RX_EULER_RAW_FLAG =    np.uint64(1 << 2)
SPI_RX_EULER_MAG_FLAG =    np.uint64(1 << 3)
SPI_RX_CUR_ALL_FLAG =      np.uint64(1 << 4)
SPI_RX_CUR_LIGHT1_FLAG =   np.uint64(1 << 5)
SPI_RX_CUR_LIGHT2_FLAG =   np.uint64(1 << 6)
SPI_RX_VOLTS24_FLAG =      np.uint64(1 << 7)
SPI_RX_MOTx_PHASE_A_FLAG = lambda x: np.uint64(1 << (8  + x))
SPI_RX_MOTx_PHASE_B_FLAG = lambda x: np.uint64(1 << (14 + x))
SPI_RX_MOTx_PHASE_C_FLAG = lambda x: np.uint64(1 << (20 + x))
SPI_RX_MAN_UNITx_VOLTAGE_FLAG =      lambda x: np.uint64(1 << (26 + x))
SPI_RX_MAN_UNITx_PHASES_A_B_FLAG =   lambda x: np.uint64(1 << (29 + x))
SPI_RX_MAN_UNITx_ANGLE_FLAG =        lambda x: np.uint64(1 << (32 + x))


SPI_TX_DES_MOTORS_FLAG =       np.uint64(1 << 0)
SPI_TX_DES_LIGHT_FLAG =        np.uint64(1 << 1)
SPI_TX_DES_CAM_ANGLE_FLAG =    np.uint64(1 << 2)
SPI_TX_DES_MAN_Qx_FLAG =       lambda x: np.uint64(1 << (3 + x))
SPI_TX_DES_MAN_GRIP_FLAG =     np.uint64(1 << 6)


baseRxPacket = "Qffffffffffffffffffffffffffffffffffffffffffffff"

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
        self.des_man_angles = [0.0]*3
        self.des_man_grip_state = GripState.UWMANIPULATOR_GRIP_STOP

        # RX
        self.euler =    [0.0, 0.0, 0.0]
        self.eulerAcc = [0.0, 0.0, 0.0]
        self.eulerRaw = [0.0, 0.0, 0.0]
        self.eulerMag = [0.0, 0.0, 0.0]
        self.currentAll = 0
        self.cirrentLight1 = 0
        self.cirrentLight2 = 0
        self.voltage24 = 0
        self.thrustersPhaseCurrents = [[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]]
        self.manVoltages = [0.0]*3
        self.manPhaseCurrents = [[0.0, 0.0],[0.0, 0.0],[0.0, 0.0]]
        self.manAngles = [0.0]*3

        self.tx_refresh_flags = np.uint64(0)
        self.rx_refresh_flags = np.uint64(0)

    def _parse_rx_buffer(self):
        if (self.rx_buffer[RxBufferOffsets.MAGIC_START] != self.MAGIC_START or
            self.rx_buffer[RxBufferOffsets.MAGIC_END] != self.MAGIC_END):
            return False
        rxPacket = 0
        try:
            rxPacket = struct.unpack_from("=" + formPacket(baseRxPacket), self.rx_buffer)
            #print(*["%.2f" % elem for elem in rxPacket], sep ='; ')
        except:
            print("SPI RX WRONG PACKET")
            return False
        
        self.rx_refresh_flags = np.uint64(rxPacket[1])
        self.euler = [rxPacket[2], rxPacket[3], rxPacket[4]] if self.rx_refresh_flags & SPI_RX_EULER_FLAG else None
        self.eulerAcc = [rxPacket[5], rxPacket[6], rxPacket[7]] if self.rx_refresh_flags & SPI_RX_EULER_ACC_FLAG else None
        self.eulerRaw = [rxPacket[8], rxPacket[9], rxPacket[10]] if self.rx_refresh_flags & SPI_RX_EULER_RAW_FLAG else None
        self.eulerMag = [rxPacket[11], rxPacket[12], rxPacket[13]] if self.rx_refresh_flags & SPI_RX_EULER_MAG_FLAG else None
        self.currentAll = rxPacket[14] if self.rx_refresh_flags & SPI_RX_CUR_ALL_FLAG else None
        self.cirrentLight1 = rxPacket[15] if self.rx_refresh_flags & SPI_RX_CUR_LIGHT1_FLAG else None
        self.cirrentLight2 = rxPacket[16] if self.rx_refresh_flags & SPI_RX_CUR_LIGHT2_FLAG else None
        self.voltage24 = rxPacket[17] if self.rx_refresh_flags & SPI_RX_VOLTS24_FLAG else None
        
        for i in range(6):
            self.thrustersPhaseCurrents[i][0] = rxPacket[18+i*3] if self.rx_refresh_flags & SPI_RX_MOTx_PHASE_A_FLAG(i) else None
            self.thrustersPhaseCurrents[i][1] = rxPacket[19+i*3] if self.rx_refresh_flags & SPI_RX_MOTx_PHASE_B_FLAG(i) else None
            self.thrustersPhaseCurrents[i][2] = rxPacket[20+i*3] if self.rx_refresh_flags & SPI_RX_MOTx_PHASE_C_FLAG(i) else None
        #     print([rxPacket[18+i*3], rxPacket[19+i*3], rxPacket[20+i*3]])
        if self.thrustersPhaseCurrents is not None:
            print(self.thrustersPhaseCurrents)
        for i in range(3):
            self.manVoltages[i] = rxPacket[36+i*4+2] if self.rx_refresh_flags & SPI_RX_MAN_UNITx_VOLTAGE_FLAG(i) else None
            self.manPhaseCurrents[i][0] = copy.deepcopy(rxPacket[36+i*4+0]) if self.rx_refresh_flags & SPI_RX_MAN_UNITx_PHASES_A_B_FLAG(i) else None
            self.manPhaseCurrents[i][1] = copy.deepcopy(rxPacket[36+i*4+1]) if self.rx_refresh_flags & SPI_RX_MAN_UNITx_PHASES_A_B_FLAG(i) else None
            # print([rxPacket[36+i*4+0] ,rxPacket[36+i*4+1]])
            self.manAngles[i] = rxPacket[36+i*4+3] if self.rx_refresh_flags & SPI_RX_MAN_UNITx_ANGLE_FLAG(i) else None 
        # print(self.manPhaseCurrents)
        return True

    def _fill_tx_buffer(self):
        self.tx_buffer[TxBufferOffsets.MAGIC_START] = self.MAGIC_START
        self.tx_buffer[TxBufferOffsets.MAGIC_END] = self.MAGIC_END

        struct.pack_into('Q', self.tx_buffer, TxBufferOffsets.FLAGS, self.tx_refresh_flags)
                       
        if self.tx_refresh_flags & SPI_TX_DES_LIGHT_FLAG:
            for index in range(0, 2):           
                offset = TxBufferOffsets.LIGHT + 4 * index
                struct.pack_into('f', self.tx_buffer, offset, self.des_lights[index])
        
        if self.tx_refresh_flags & SPI_TX_DES_MOTORS_FLAG:
            for index in range(0, 6):           
                offset = TxBufferOffsets.MOTORS + 4 * index
                struct.pack_into('f', self.tx_buffer, offset, self.des_mot[index])
        
        if self.tx_refresh_flags & SPI_TX_DES_CAM_ANGLE_FLAG:
            offset = TxBufferOffsets.CAM_ANGLE
            struct.pack_into('f', self.tx_buffer, offset, self.des_cam_angle)
        
        for index in range(3):
            if self.tx_refresh_flags & SPI_TX_DES_MAN_Qx_FLAG(index):
                offset = TxBufferOffsets.MAN_ANGLES + 4 * index
                struct.pack_into('f', self.tx_buffer, offset, self.des_man_angles[index])
        
        if self.tx_refresh_flags & SPI_TX_DES_MAN_GRIP_FLAG:
            offset = TxBufferOffsets.MAN_GRIP
            struct.pack_into('B', self.tx_buffer, offset, self.des_man_grip_state)

    def transfer(self):
        self._fill_tx_buffer()
        # received = struct.unpack_from("QffffffffffffB", self.tx_buffer,1)
        # print(*["%.2f" % elem for elem in received], sep ='; ')
        count, self.rx_buffer = self.pi.spi_xfer(self.spi_handle, self.tx_buffer)
        self.tx_refresh_flags = np.uint64(0)
        return self._parse_rx_buffer()

    # Setters
    def set_mots_values(self, values):
        self.des_mot = values
        self.tx_refresh_flags |= SPI_TX_DES_MOTORS_FLAG
    
    def set_lights_values(self, values):
        self.des_lights = values
        self.tx_refresh_flags |= SPI_TX_DES_LIGHT_FLAG
        
    def set_cam_angle_value(self, value):
        self.des_cam_angle = value
        self.tx_refresh_flags |= SPI_TX_DES_CAM_ANGLE_FLAG

    def set_man_angle_value(self, index, value):
        if not index in range(3):
            return
        self.des_man_angles[index] = value
        self.tx_refresh_flags |= SPI_TX_DES_MAN_Qx_FLAG(index)

    def set_man_grip_value(self, value):
        self.des_man_grip_state = np.uint8(value)
        self.tx_refresh_flags |= SPI_TX_DES_MAN_GRIP_FLAG 

    # Getters
    def get_IMU_angles(self):
        if self.rx_refresh_flags & SPI_RX_EULER_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_EULER_FLAG)
            return self.euler
        return None
    
    def get_IMU_raw(self):
        if self.rx_refresh_flags & SPI_RX_EULER_RAW_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_EULER_RAW_FLAG)
            return self.eulerRaw
        return None
    
    def get_IMU_accelerometer(self):
        if self.rx_refresh_flags & SPI_RX_EULER_ACC_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_EULER_ACC_FLAG)
            return self.eulerAcc
        return None
    
    def get_IMU_magnetometer(self):
        if self.rx_refresh_flags & SPI_RX_EULER_MAG_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_EULER_MAG_FLAG)
            return self.eulerMag
        return None
    
    def get_current_all(self):
        if self.rx_refresh_flags & SPI_RX_CUR_ALL_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_CUR_ALL_FLAG)
            return self.currentAll
        return None
    
    def get_current_lights(self):
        light1 = None
        light2 = None
        if self.rx_refresh_flags & SPI_RX_CUR_LIGHT1_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_CUR_LIGHT1_FLAG)
            light1 =  self.cirrentLight1
            
        if self.rx_refresh_flags & SPI_RX_CUR_LIGHT2_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_CUR_LIGHT2_FLAG)
            light2 =  self.cirrentLight2
        
        if light1 is not None or light2 is not None:
            return [light1, light2]            
        return None
    
    def get_voltage(self):
        if self.rx_refresh_flags & SPI_RX_VOLTS24_FLAG:
            self.rx_refresh_flags &= ~(SPI_RX_VOLTS24_FLAG)
            return self.voltage24
        return None

    def get_thrusters_phase_current(self, index, phase: str):
        if not index in range(6):
            return None
        if not phase in ('A', 'B', 'C'):
            return None
        if phase == 'A':
            if self.rx_refresh_flags & SPI_RX_MOTx_PHASE_A_FLAG(index):
                self.rx_refresh_flags &= ~(SPI_RX_MOTx_PHASE_A_FLAG(index))
                return self.thrustersPhaseCurrents[index][0]
        if phase == 'B':
            if self.rx_refresh_flags & SPI_RX_MOTx_PHASE_B_FLAG(index):
                self.rx_refresh_flags &= ~(SPI_RX_MOTx_PHASE_B_FLAG(index))
                return self.thrustersPhaseCurrents[index][1]
        if phase == 'C':
            if self.rx_refresh_flags & SPI_RX_MOTx_PHASE_C_FLAG(index):
                self.rx_refresh_flags &= ~(SPI_RX_MOTx_PHASE_C_FLAG(index))
                return self.thrustersPhaseCurrents[index][2]
        return None
    
    def get_man_voltage(self, index):
        if not index in range(3):
            return None
        if self.rx_refresh_flags & SPI_RX_MAN_UNITx_VOLTAGE_FLAG(index):
            self.rx_refresh_flags &= ~(SPI_RX_MAN_UNITx_VOLTAGE_FLAG(index))
            return self.manVoltages[index]
        return None
    
    def get_man_phase_currents(self, index):
        if not index in range(3):
            return None
        if self.rx_refresh_flags & SPI_RX_MAN_UNITx_PHASES_A_B_FLAG(index):
            self.rx_refresh_flags &= ~(SPI_RX_MAN_UNITx_PHASES_A_B_FLAG(index))
            return self.manPhaseCurrents[index]
        return None
    
    def get_man_angles(self, index):
        if not index in range(3):
            return None
        if self.rx_refresh_flags & SPI_RX_MAN_UNITx_ANGLE_FLAG(index):
            self.rx_refresh_flags &= ~(SPI_RX_MAN_UNITx_ANGLE_FLAG(index))
            return self.manAngles[index]

    def close(self):
        self.pi.spi_close(self.spi_handle)
        self.pi.stop()
