BUFFER_SIZE_DEFAULT = 256

# *************************************************************
# LoRa Modem Settings
# *************************************************************
#
# LoRa spreading factors
LORA_SF5 = 0x50
LORA_SF6 = 0x60
LORA_SF7 = 0x70
LORA_SF8 = 0x80
LORA_SF9 = 0x90
LORA_SF10 = 0xA0
LORA_SF11 = 0xB0
LORA_SF12 = 0xC0

# LoRa bandwidths
LORA_BW_0200 = 0x34  # actually 203125hz
LORA_BW_0400 = 0x26  # actually 406250hz
LORA_BW_0800 = 0x18  # actually 812500hz
LORA_BW_1600 = 0x0A  # actually 1625000hz

# LoRa coding rates
LORA_CR_4_5 = 0x01
LORA_CR_4_6 = 0x02
LORA_CR_4_7 = 0x03
LORA_CR_4_8 = 0x04

# March 2020  A new interleaving scheme has been implemented to increase robustness to
# burst interference and/or strong Doppler events.
# Note: There is a limitation on maximum payload length for LORA_CR_LI_4_8. Payload length should
# not exceed 253 bytes if CRC is enabled.
LORA_CR_LI_4_5 = 0x05
LORA_CR_LI_4_6 = 0x06
LORA_CR_LI_4_8 = 0x07

# LoRa CAD settings
LORA_CAD_01_SYMBOL = 0x00
LORA_CAD_02_SYMBOL = 0x20
LORA_CAD_04_SYMBOL = 0x40
LORA_CAD_08_SYMBOL = 0x60
LORA_CAD_16_SYMBOL = 0x80

# LoRa Header Types
LORA_PACKET_VARIABLE_LENGTH = 0x00
LORA_PACKET_FIXED_LENGTH = 0x80
LORA_PACKET_EXPLICIT = LORA_PACKET_VARIABLE_LENGTH
LORA_PACKET_IMPLICIT = LORA_PACKET_FIXED_LENGTH

# LoRa packet CRC settings
LORA_CRC_ON = 0x20
LORA_CRC_OFF = 0x00

# LoRa IQ Setings
LORA_IQ_NORMAL = 0x40
LORA_IQ_INVERTED = 0x00


FREQ_STEP = 198.364
FREQ_ERROR_CORRECTION = 1.55


# *************************************************************
#  SX1280 Interrupt flags
# *************************************************************

IRQ_RADIO_NONE = 0x0000
IRQ_TX_DONE = 0x0001
IRQ_RX_DONE = 0x0002
IRQ_SYNCWORD_VALID = 0x0004
IRQ_SYNCWORD_ERROR = 0x0008
IRQ_HEADER_VALID = 0x0010
IRQ_HEADER_ERROR = 0x0020
IRQ_CRC_ERROR = 0x0040
IRQ_RANGING_SLAVE_RESPONSE_DONE = 0x0080

IRQ_RANGING_SLAVE_REQUEST_DISCARDED = 0x0100
IRQ_RANGING_MASTER_RESULT_VALID = 0x0200
IRQ_RANGING_MASTER_RESULT_TIMEOUT = 0x0400
IRQ_RANGING_SLAVE_REQUEST_VALID = 0x0800
IRQ_CAD_DONE = 0x1000
IRQ_CAD_ACTIVITY_DETECTED = 0x2000
IRQ_RX_TX_TIMEOUT = 0x4000
IRQ_TX_TIMEOUT = 0x4000
IRQ_RX_TIMEOUT = 0x4000
IRQ_PREAMBLE_DETECTED = 0x8000
IRQ_RADIO_ALL = 0xFFFF


# *************************************************************
#  SX1280 Commands
# *************************************************************

RADIO_GET_PACKETTYPE = 0x03
RADIO_GET_IRQSTATUS = 0x15
RADIO_GET_RXBUFFERSTATUS = 0x17
RADIO_WRITE_REGISTER = 0x18
RADIO_READ_REGISTER = 0x19
RADIO_WRITE_BUFFER = 0x1A
RADIO_READ_BUFFER = 0x1B
RADIO_GET_PACKETSTATUS = 0x1D
RADIO_GET_RSSIINST = 0x1F
RADIO_SET_STANDBY = 0x80
RADIO_SET_RX = 0x82
RADIO_SET_TX = 0x83
RADIO_SET_SLEEP = 0x84
RADIO_SET_RFFREQUENCY = 0x86
RADIO_SET_CADPARAMS = 0x88
RADIO_CALIBRATE = 0x89
RADIO_SET_PACKETTYPE = 0x8A
RADIO_SET_MODULATIONPARAMS = 0x8B
RADIO_SET_PACKETPARAMS = 0x8C
RADIO_SET_DIOIRQPARAMS = 0x8D
RADIO_SET_TXPARAMS = 0x8E
RADIO_SET_BUFFERBASEADDRESS = 0x8F
RADIO_SET_RXDUTYCYCLE = 0x94
RADIO_SET_REGULATORMODE = 0x96
RADIO_CLR_IRQSTATUS = 0x97
RADIO_SET_AUTOTX = 0x98
RADIO_SET_LONGPREAMBLE = 0x9B
RADIO_SET_UARTSPEED = 0x9D
RADIO_SET_AUTOFS = 0x9E
RADIO_SET_RANGING_ROLE = 0xA3
RADIO_GET_STATUS = 0xC0
RADIO_SET_FS = 0xC1
RADIO_SET_CAD = 0xC5
RADIO_SET_TXCONTINUOUSWAVE = 0xD1
RADIO_SET_TXCONTINUOUSPREAMBLE = 0xD2
RADIO_SET_SAVECONTEXT = 0xD5


# *************************************************************
#  SX1280 Registers
# *************************************************************

REG_LNA_REGIME = 0x0891
REG_LR_PAYLOADLENGTH = 0x901
REG_LR_PACKETPARAMS = 0x903

REG_RFFrequency23_16 = 0x906
REG_RFFrequency15_8 = 0x907
REG_RFFrequency7_0 = 0x908

REG_FLRC_RFFrequency23_16 = 0x9A3  # found by experiment
REG_FLRC_RFFrequency15_8 = 0x9A4
REG_FLRC_RFFrequency7_0 = 0x9A5

REG_RANGING_FILTER_WINDOW_SIZE = 0x091E
REG_LR_DEVICERANGINGADDR = 0x0916
REG_LR_DEVICERANGINGADDR = 0x0916
REG_LR_RANGINGRESULTCONFIG = 0x0924
REG_LR_RANGINGRERXTXDELAYCAL = 0x092C
REG_LR_RANGINGIDCHECKLENGTH = 0x0931
REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB = 0x954
REG_LR_ESTIMATED_FREQUENCY_ERROR_MID = 0x955
REG_LR_ESTIMATED_FREQUENCY_ERROR_LSB = 0x956
REG_LR_RANGINGRESULTBASEADDR = 0x0961
REG_RANGING_RSSI = 0x0964
REG_LR_FLRCPAYLOADLENGTH = 0x09C3  # found by experiment
REG_LR_SYNCWORDTOLERANCE = 0x09CD
REG_LR_SYNCWORDBASEADDRESS1 = 0x09CE
REG_FLRCSYNCWORD1_BASEADDR = 0x09CF
REG_LR_SYNCWORDBASEADDRESS2 = 0x09D3
REG_FLRCSYNCWORD2_BASEADDR = 0x09D4
REG_LR_SYNCWORDBASEADDRESS3 = 0x09D8
REG_FLRCSYNCWORD3_BASEADDR = 0x09D9

REG_RANGING_RSSI = 0x0964

REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK = 0x0FFFFF

# SX1280 Packet Types
PACKET_TYPE_GFSK = 0x00
PACKET_TYPE_LORA = 0x01
PACKET_TYPE_RANGING = 0x02
PACKET_TYPE_FLRC = 0x03
PACKET_TYPE_BLE = 0x04

# SX1280 Standby modes
MODE_STDBY_RC = 0x00
MODE_STDBY_XOSC = 0x01

# TX and RX timeout based periods
PERIODBASE_15_US = 0x00
PERIODBASE_62_US = 0x01
PERIODBASE_01_MS = 0x02
PERIODBASE_04_MS = 0x03

# TX ramp periods
RADIO_RAMP_02_US = 0x00
RADIO_RAMP_04_US = 0x20
RADIO_RAMP_06_US = 0x40
RADIO_RAMP_08_US = 0x60
RADIO_RAMP_10_US = 0x80
RADIO_RAMP_12_US = 0xA0
RADIO_RAMP_16_US = 0xC0
RADIO_RAMP_20_US = 0xE0

# SX1280 Power settings
USE_LDO = 0x00
USE_DCDC = 0x01


# *************************************************************
# SX1280 Ranging settings
# *************************************************************

RANGING_IDCHECK_LENGTH_08_BITS = 0x00
RANGING_IDCHECK_LENGTH_16_BITS = 0x01
RANGING_IDCHECK_LENGTH_24_BITS = 0x02
RANGING_IDCHECK_LENGTH_32_BITS = 0x03

RANGING_RESULT_RAW = 0x00
RANGING_RESULT_AVERAGED = 0x01
RANGING_RESULT_DEBIASED = 0x02
RANGING_RESULT_FILTERED = 0x03


MASK_RANGINGMUXSEL = 0xCF

RANGING_SLAVE = 0x00
RANGING_MASTER = 0x01


# *************************************************************
# GFSK  modem settings
# *************************************************************

GFS_BLE_BR_2_000_BW_2_4 = 0x04
GFS_BLE_BR_1_600_BW_2_4 = 0x28
GFS_BLE_BR_1_000_BW_2_4 = 0x4C
GFS_BLE_BR_1_000_BW_1_2 = 0x45
GFS_BLE_BR_0_800_BW_2_4 = 0x70
GFS_BLE_BR_0_800_BW_1_2 = 0x69
GFS_BLE_BR_0_500_BW_1_2 = 0x8D
GFS_BLE_BR_0_500_BW_0_6 = 0x86
GFS_BLE_BR_0_400_BW_1_2 = 0xB1
GFS_BLE_BR_0_400_BW_0_6 = 0xAA
GFS_BLE_BR_0_250_BW_0_6 = 0xCE
GFS_BLE_BR_0_250_BW_0_3 = 0xC7
GFS_BLE_BR_0_125_BW_0_3 = 0xEF

GFS_BLE_MOD_IND_0_35 = 0
GFS_BLE_MOD_IND_0_50 = 1
GFS_BLE_MOD_IND_0_75 = 2
GFS_BLE_MOD_IND_1_00 = 3
GFS_BLE_MOD_IND_1_25 = 4
GFS_BLE_MOD_IND_1_50 = 5
GFS_BLE_MOD_IND_1_75 = 6
GFS_BLE_MOD_IND_2_00 = 7
GFS_BLE_MOD_IND_2_25 = 8
GFS_BLE_MOD_IND_2_50 = 9
GFS_BLE_MOD_IND_2_75 = 10
GFS_BLE_MOD_IND_3_00 = 11
GFS_BLE_MOD_IND_3_25 = 12
GFS_BLE_MOD_IND_3_50 = 13
GFS_BLE_MOD_IND_3_75 = 14
GFS_BLE_MOD_IND_4_00 = 15

PREAMBLE_LENGTH_04_BITS = 0x00  # 4 bits
PREAMBLE_LENGTH_08_BITS = 0x10  # 8 bits
PREAMBLE_LENGTH_12_BITS = 0x20  # 12 bits
PREAMBLE_LENGTH_16_BITS = 0x30  # 16 bits
PREAMBLE_LENGTH_20_BITS = 0x40  # 20 bits
PREAMBLE_LENGTH_24_BITS = 0x50  # 24 bits
PREAMBLE_LENGTH_28_BITS = 0x60  # 28 bits
PREAMBLE_LENGTH_32_BITS = 0x70  # 32 bits

GFS_SYNCWORD_LENGTH_1_BYTE = 0x00  # Sync word length 1 byte
GFS_SYNCWORD_LENGTH_2_BYTE = 0x02  # Sync word length 2 bytes
GFS_SYNCWORD_LENGTH_3_BYTE = 0x04  # Sync word length 3 bytes
GFS_SYNCWORD_LENGTH_4_BYTE = 0x06  # Sync word length 4 bytes
GFS_SYNCWORD_LENGTH_5_BYTE = 0x08  # Sync word length 5 bytes

RADIO_RX_MATCH_SYNCWORD_OFF = 0x00  # no search for SyncWord
RADIO_RX_MATCH_SYNCWORD_1 = 0x10
RADIO_RX_MATCH_SYNCWORD_2 = 0x20
RADIO_RX_MATCH_SYNCWORD_1_2 = 0x30
RADIO_RX_MATCH_SYNCWORD_3 = 0x40
RADIO_RX_MATCH_SYNCWORD_1_3 = 0x50
RADIO_RX_MATCH_SYNCWORD_2_3 = 0x60
RADIO_RX_MATCH_SYNCWORD_1_2_3 = 0x70

RADIO_PACKET_FIXED_LENGTH = (
    0x00  # The packet is fixed length, klnown on both RX and TX, no header
)
RADIO_PACKET_VARIABLE_LENGTH = 0x20  # The packet is variable size, header included

RADIO_CRC_OFF = 0x00
RADIO_CRC_1_BYTES = 0x10
RADIO_CRC_2_BYTES = 0x20
RADIO_CRC_3_BYTES = 0x30

RADIO_WHITENING_ON = 0x00
RADIO_WHITENING_OFF = 0x08

# End GFSK ****************************************************


# *************************************************************
# FLRC  modem settings
# *************************************************************

FLRC_SYNC_NOSYNC = 0x00
FLRC_SYNC_WORD_LEN_P32S = 0x04

RX_DISABLE_SYNC_WORD = 0x00
RX_MATCH_SYNC_WORD_1 = 0x10
RX_MATCH_SYNC_WORD_2 = 0x20
RX_MATCH_SYNC_WORD_1_2 = 0x30
RX_MATCH_SYNC_WORD_3 = 0x40
RX_MATCH_SYNC_WORD_1_3 = 0x50
RX_MATCH_SYNC_WORD_2_3 = 0x60
RX_MATCH_SYNC_WORD_1_2_3 = 0x70

FLRC_BR_1_300_BW_1_2 = 0x45  # 1.3Mbs
FLRC_BR_1_000_BW_1_2 = 0x69  # 1.04Mbs
FLRC_BR_0_650_BW_0_6 = 0x86  # 0.65Mbs
FLRC_BR_0_520_BW_0_6 = 0xAA  # 0.52Mbs
FLRC_BR_0_325_BW_0_3 = 0xC7  # 0.325Mbs
FLRC_BR_0_260_BW_0_3 = 0xEB  # 0.26Mbs

FLRC_CR_1_2 = 0x00  # coding rate 1:2
FLRC_CR_3_4 = 0x02  # coding rate 3:4
FLRC_CR_1_0 = 0x04  # coding rate 1

BT_DIS = 0x00  # No filtering
BT_1 = 0x10  # 1
BT_0_5 = 0x20  # 0.5

RADIO_MOD_SHAPING_BT_OFF = 0x00
RADIO_MOD_SHAPING_BT_1_0 = 0x10
RADIO_MOD_SHAPING_BT_0_5 = 0x20


# Table 13-45: PacketStatus2 in FLRC Packet
PacketCtrlBusy = 0x01
PacketReceived = 0x02
HeaderReceived = 0x04
AbortError = 0x08
CrcError = 0x10
LengthError = 0x20
SyncError = 0x40
Reserved = 0x80


# Table 13-46: PacketStatus3 in FLRC Packet
PktSent = 0x01
rxpiderr = 0x08
rx_no_ack = 0x10

# FLRC default packetparamns
FLRC_Default_AGCPreambleLength = PREAMBLE_LENGTH_32_BITS  # packetParam1
FLRC_Default_SyncWordLength = FLRC_SYNC_WORD_LEN_P32S  # packetParam2
FLRC_Default_SyncWordMatch = RADIO_RX_MATCH_SYNCWORD_1  # packetParam3
FLRC_Default_PacketType = RADIO_PACKET_VARIABLE_LENGTH  # packetParam4
FLRC_Default_PayloadLength = BUFFER_SIZE_DEFAULT  # packetParam5
FLRC_Default_CrcLength = RADIO_CRC_3_BYTES  # packetParam6
FLRC_Default_Whitening = RADIO_WHITENING_OFF  # packetParam7


# Table 11-15 Sleep modes
RETAIN_INSTRUCTION_RAM = 0x04
RETAIN_DATABUFFER = 0x02
RETAIN_DATA_RAM = 0x01
CONFIGURATION_RETENTION = 0x01  # included for libray compatibility
RETAIN_None = 0x00


# ifndef RAMP_TIME
RAMP_TIME = RADIO_RAMP_02_US
# endif

# ifndef PERIODBASE
PERIODBASE = PERIODBASE_01_MS
# endif

# ifndef PERIODBASE_COUNT_15_8
PERIODBASE_COUNT_15_8 = 0
# endif

# ifndef PERIODBASE_COUNT_7_0
PERIODBASE_COUNT_7_0 = 0
# endif


DEVICE_SX1280 = 0x20
DEVICE_SX1281 = 0x21


# SPI settings
LTspeedMaximum = 8000000
# LTdataOrder = MSBFIRST
# LTdataMode = SPI_MODE0

RANGING_VALID = 0x03
RANGING_TIMEOUT = 0x02
WAIT_RX = 0x01
WAIT_TX = 0x01
NO_WAIT = 0x00

CalibrationSF10BW400 = 10180  # calibration value for ranging, SF10, BW400
CalibrationSF5BW1600 = 13100  # calibration value for ranging, SF5, BW1600


RNG_CALIB_0400 = [10260, 10244, 10228, 10212, 10196, 10180]  # SF5 to SF10
RNG_CALIB_0800 = [11380, 11370, 11360, 11350, 11340, 11330]
RNG_CALIB_1600 = [13100, 13160, 13220, 13280, 13340, 13400]


# These are the bit numbers which when set indicate reliable errors, variable _ReliableErrors
ReliableCRCError = (
    0x00  # bit number set in _ReliableErrors when there is a reliable CRC missmatch
)
ReliableIDError = (
    0x01  # bit number set in _ReliableErrors when there is a NetworkID missmatch
)
ReliableSizeError = (
    0x02  # bit number set in _ReliableErrors when there is a size error for packet
)
ReliableACKError = 0x03  # bit number set in _ReliableErrors when there is a ACK error
ReliableTimeout = (
    0x04  # bit number set in _ReliableErrors when there is a timeout error
)
SegmentSequenceError = (
    0x05  # bit number set in _ReliableErrors when there is a segment sequence error
)
FileError = 0x06  # bit number set in _ReliableErrors when there ia a file (SD) error

# These are the bit numbers which when set indicate reliable status flags, variable _ReliableFlags
ReliableACKSent = 0x00  # bit number set in _ReliableFlags when there is a ACK sent
ReliableACKReceived = (
    0x01  # bit number set in _ReliableFlags when there is a ACK received
)

# These are the bit numbers which when set indicate reliable configuration, variable _ReliableConfig
NoReliableCRC = 0x00  # bit number set in _ReliableConfig when reliable CRC is not used
NoAutoACK = 0x01  # bit number set in _ReliableConfig when ACK is not used


# number of bits to use to match ranging slave address
bits8 = 0x00
bits16 = 0x40
bits24 = 0x80
bits32 = 0xC0

_PERIODBASE = PERIODBASE_01_MS

IRQ1_DEF = (
    "RngSlaveReqDiscard",
    "RngMasterResultValid",
    "RngMasterTimeout",
    "RngMasterReqValid",
    "CadDone",
    "CadDetected",
    "RxTxTimeout",
    "AdvRngDone",
)

IRQ2_DEF = (
    "TxDone",
    "RxDone",
    "SyncWrdValid",
    "SyncWrdErr",
    "HeaderValid",
    "HeaderErr",
    "CrcErr",
    "RngSlaveResponseDone",
)

STATUS_MODE = {
    0: "N/A",
    1: "N/A",
    2: "STDBY_RC",
    3: "STDBY_XOSC",
    4: "FS",
    5: "Rx",
    6: "Tx",
}

STATUS_CMD = {
    0: "N/A",
    1: "Cmd Successful",
    2: "Data Available",
    3: "Timed-out",
    4: "Cmd Error",
    5: "Failure to Execute Cmd",
    6: "Tx Done",
}
