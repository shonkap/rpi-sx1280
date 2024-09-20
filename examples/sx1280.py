import time
from typing import Optional
import RPi.GPIO as GPIO
from loguru import logger
import const
from newio import SpiDevice, IOPin


class SX128XLT:
    _device: SpiDevice
    _spiNSS: IOPin
    _rfBusy: IOPin
    _nReset: Optional[IOPin]

    _txEn: Optional[IOPin]
    _rxEn: Optional[IOPin]

    _dio1: Optional[IOPin]
    _dio2: Optional[IOPin]
    _dio3: Optional[IOPin]

    _TXDonePin: Optional[IOPin]
    _RXDonePin: Optional[IOPin]

    _rxtxpinmode: bool = False
    _deviceConnected: bool = False

    def __init__(
        self,
        spi_idx: int,
        spi_cs: int,
        spi_nss: int,
        pin_rfbusy: int,
        pin_nreset=None,
        pin_dio1=None,
        pin_dio2=None,
        pin_dio3=None,
        pin_rxen=None,
        pin_txen=None,
        device=const.DEVICE_SX1280,
    ):
        self._device_ID = device
        self._device = SpiDevice(spi_idx, spi_cs)
        self._spiNSS = IOPin(spi_nss, GPIO.OUT, GPIO.HIGH)

        self._rfBusy = IOPin(pin_rfbusy, GPIO.IN)

        if pin_nreset is not None:
            self._nReset = IOPin(pin_nreset, GPIO.OUT, GPIO.HIGH)
            self._nReset.low()

        if pin_dio1:
            self._dio1 = IOPin(pin_dio1, GPIO.IN)

            self._TXDonePin = self._dio1
            self._RXDonePin = self._dio1

        if pin_dio2:
            self._dio2 = IOPin(pin_dio2, GPIO.IN)

        if pin_dio3:
            self._dio3 = IOPin(pin_dio3, GPIO.IN)

        if pin_rxen and pin_txen:
            self._rxEn = IOPin(pin_rxen, GPIO.OUT, GPIO.LOW)
            self._txEn = IOPin(pin_txen, GPIO.OUT, GPIO.LOW)
            self._rxtxpinmode = True
        else:
            self._rxtxpinmode = False

        self.resetDevice()

    def isConnected(self):
        return self._deviceConnected

    def rxEnable(self):
        if self._rxEn:
            self._rxEn.high()
            self._txEn.low()

    def txEnable(self):
        if self._txEn:
            self._rxEn.low()
            self._txEn.high()

    def checkBusy(self):
        startTimeMs = time.time()

        while self._rfBusy.read() == GPIO.HIGH:
            if time.time() - startTimeMs > 9:  # will wait 10mS for busy to complete
                logger.error("SX1280 busy timeout")

                self.resetDevice()
                self.setMode(const.MODE_STDBY_RC)
                self.config()

                break

    def config(self):
        self.resetDevice()
        self.setMode(const.MODE_STDBY_RC)
        self.setRegulatorMode(self.savedRegulatorMode)
        self.setPacketType(self.savedPacketType)
        self.setRfFrequency(self.savedFrequency, self.savedOffset)
        self.setModulationParams(
            self.savedModParam1, self.savedModParam2, self.savedModParam3
        )
        self.setPacketParams(
            self.savedPacketParam1,
            self.savedPacketParam2,
            self.savedPacketParam3,
            self.savedPacketParam4,
            self.savedPacketParam5,
            self.savedPacketParam6,
            self.savedPacketParam7,
        )
        self.setDioIrqParams(
            self.savedIrqMask,
            self.savedDio1Mask,
            self.savedDio2Mask,
            self.savedDio3Mask,
        )  # set for IRQ on RX done on DIO1
        self.setHighSensitivity()
        return True

    def setupLoRa(
        self,
        frequency: int,
        offset: int,
        modParam1: int,
        modParam2: int,
        modParam3: int,
    ):
        self.setMode(const.MODE_STDBY_RC)
        self.setRegulatorMode(const.USE_LDO)
        self.setPacketType(const.PACKET_TYPE_LORA)
        self.setRfFrequency(frequency, offset)
        self.setBufferBaseAddress(0, 0)
        self.setModulationParams(modParam1, modParam2, modParam3)
        self.setPacketParams(
            12,
            const.LORA_PACKET_VARIABLE_LENGTH,
            255,
            const.LORA_CRC_ON,
            const.LORA_IQ_NORMAL,
            0,
            0,
        )
        self.setDioIrqParams(
            const.IRQ_RADIO_ALL, (const.IRQ_TX_DONE + const.IRQ_RX_TX_TIMEOUT), 0, 0
        )
        self.setHighSensitivity()

        self._deviceConnected = self.checkDevice()

        if not self._deviceConnected:
            logger.error("Device Not Found")

    def setupFLRC(
        self,
        frequency: int,
        offset: int,
        modParam1: int,
        modParam2: int,
        modParam3: int,
        syncword: int,
    ):
        self.setMode(const.MODE_STDBY_RC)
        self.setRegulatorMode(const.USE_LDO)
        self.setPacketType(const.PACKET_TYPE_FLRC)
        self.setRfFrequency(frequency, offset)
        self.setBufferBaseAddress(0, 0)
        self.setModulationParams(modParam1, modParam2, modParam3)
        self.setPacketParams(
            const.PREAMBLE_LENGTH_32_BITS,
            const.FLRC_SYNC_WORD_LEN_P32S,
            const.RADIO_RX_MATCH_SYNCWORD_1,
            const.RADIO_PACKET_VARIABLE_LENGTH,
            127,
            const.RADIO_CRC_3_BYTES,
            const.RADIO_WHITENING_OFF,
        )
        self.setDioIrqParams(
            const.IRQ_RADIO_ALL, (const.IRQ_TX_DONE + const.IRQ_RX_TX_TIMEOUT), 0, 0
        )
        self.setSyncWord1(syncword)
        self.setHighSensitivity()

    def readRegisters(self, address: int, length: int) -> list[int]:
        addr_h = (address >> 8) & 0xFF
        addr_l = address & 0xFF

        self.checkBusy()

        with self._device as spi:
            bts = [const.RADIO_READ_REGISTER, addr_h, addr_l, 0xFF] + [0xFF] * length
            buffer = spi.xfer3(bts)
            buffer = buffer[4:]

            hex_str = ":".join([f"{b:02X}" for b in bts])
            hex_rec = ":".join([f"{b:02X}" for b in buffer])
            logger.debug(f"readRegisters <> {hex_str} -> {hex_rec}")

        return buffer

    def readRegister(self, address: int) -> int:
        return self.readRegisters(address, 1)[0]

    def writeRegisters(self, address: int, data: list[int]):
        addr_h = (address >> 8) & 0xFF
        addr_l = address & 0xFF

        self.checkBusy()

        with self._device as spi:
            self._spiNSS.Low()
            spi.writebytes([const.RADIO_WRITE_REGISTER, addr_h, addr_l])
            buffer = spi.xfer3(data)
            self._spiNSS.high()

            hex_str = ":".join(
                [
                    f"{b:02X}"
                    for b in [const.RADIO_WRITE_REGISTER, addr_h, addr_l] + data
                ]
            )
            hex_rec = ":".join([f"{b:02X}" for b in buffer])
            logger.debug(f"writeRegisters <> {hex_str} -> {hex_rec}")

    def writeRegister(self, address: int, data: int):
        self.writeRegisters(address, [data])

    def writeCommand(self, opCode: int, data: list[int]):
        self.checkBusy()

        with self._device as spi:
            self._spiNSS.low()
            spi.writebytes([opCode])
            spi.writebytes(data)
            self._spiNSS.high()

            hex_str = ":".join([f"{b:02X}" for b in [opCode] + data])
            logger.debug(f"writeCommand -> {hex_str}")

        if opCode != const.RADIO_SET_SLEEP:
            self.checkBusy()

    def readCommand(self, opCode: int, length: int) -> list[int]:
        self.checkBusy()

        with self._device as spi:
            self._spiNSS.low()
            spi.writebytes([opCode])
            spi.writebytes([0xFF])

            bts = [0xFF] * length
            buffer = spi.xfer3(bts)
            self._spiNSS.high()

            hex_str = ":".join([f"{b:02X}" for b in [opCode, 0xFF]])
            hex_rec = ":".join([f"{b:02X}" for b in buffer])
            logger.debug(f"readCommand <> {hex_str} -> {hex_rec}")

        return buffer

    def resetDevice(self):
        if self._nReset:
            time.sleep(0.02)
            self._nReset.low()
            time.sleep(0.05)
            self._nReset.high()
            time.sleep(0.02)

    def checkDevice(self):
        self._spiNSS.low()
        regData1 = self.readRegister(0x0908)
        self.writeRegister(0x0908, regData1 + 1)
        regData2 = self.readRegister(0x0908)
        self.writeRegister(0x0908, regData1)
        self._spiNSS.high()

        return regData2 == (regData1 + 1)

    def setMode(self, modeConfig: int):
        opcode = 0x80

        self.checkBusy()

        with self._device as spi:
            self._spiNSS.low()
            spi.writebytes([opcode])
            spi.writebytes([modeConfig])
            self._spiNSS.high()

            hex_str = f": {opcode:02X} {modeConfig:02X}"
            logger.debug(f"setMode -> {hex_str}")

        self._operatingMode = modeConfig

    def setRegulatorMode(self, mode: int):
        self.savedRegulatorMode = mode

        self.writeCommand(const.RADIO_SET_REGULATORMODE, [mode])

    def setPacketType(self, packetType: int):
        self.savedPacketType = packetType

        self.writeCommand(const.RADIO_SET_PACKETTYPE, [packetType])

    def setRfFrequency(self, frequency: int, offset: int):
        self.savedFrequency = frequency
        self.savedOffset = offset

        frequency = frequency + offset
        freqtemp = int(frequency / const.FREQ_STEP)

        logger.debug(f"setRfFrequency: {freqtemp}")
        self.writeCommand(
            const.RADIO_SET_RFFREQUENCY,
            [
                int((freqtemp >> 16) & 0xFF),
                int((freqtemp >> 8) & 0xFF),
                int(freqtemp & 0xFF),
            ],
        )

    def setBufferBaseAddress(self, txBaseAddress: int, rxBaseAddress: int):
        logger.debug(f"setBufferBaseAddress: {txBaseAddress}, {rxBaseAddress}")
        self.writeCommand(
            const.RADIO_SET_BUFFERBASEADDRESS,
            [
                txBaseAddress,
                rxBaseAddress,
            ],
        )

    def setModulationParams(self, modParam1: int, modParam2: int, modParam3: int):
        self.savedModParam1 = modParam1
        self.savedModParam2 = modParam2
        self.savedModParam3 = modParam3

        self.writeCommand(
            const.RADIO_SET_MODULATIONPARAMS,
            [
                modParam1,
                modParam2,
                modParam3,
            ],
        )

        # implement data sheet additions, datasheet SX1280-1_V3.2section 14.47

        self.writeRegister(0x93C, 0x1)

        if modParam1 == const.LORA_SF5:
            self.writeRegister(0x925, 0x1E)
        elif modParam1 == const.LORA_SF6:
            self.writeRegister(0x925, 0x1E)
        elif modParam1 == const.LORA_SF7:
            self.writeRegister(0x925, 0x37)
        elif modParam1 == const.LORA_SF8:
            self.writeRegister(0x925, 0x37)
        elif modParam1 == const.LORA_SF9:
            self.writeRegister(0x925, 0x32)
        elif modParam1 == const.LORA_SF10:
            self.writeRegister(0x925, 0x32)
        elif modParam1 == const.LORA_SF11:
            self.writeRegister(0x925, 0x32)
        elif modParam1 == const.LORA_SF12:
            self.writeRegister(0x925, 0x32)

    def setPacketParams(
        self,
        packetParam1: int,
        packetParam2: int,
        packetParam3: int,
        packetParam4: int,
        packetParam5: int,
        packetParam6: Optional[int] = None,
        packetParam7: Optional[int] = None,
    ):
        self.savedPacketParam1 = packetParam1
        self.savedPacketParam2 = packetParam2
        self.savedPacketParam3 = packetParam3
        self.savedPacketParam4 = packetParam4
        self.savedPacketParam5 = packetParam5

        if packetParam6 is not None:
            self.savedPacketParam6 = packetParam6
        if packetParam7 is not None:
            self.savedPacketParam7 = packetParam7

        if packetParam6 is not None and packetParam7 is not None:
            self.writeCommand(
                const.RADIO_SET_PACKETPARAMS,
                [
                    packetParam1,
                    packetParam2,
                    packetParam3,
                    packetParam4,
                    packetParam5,
                    packetParam6,
                    packetParam7,
                ],
            )
        else:
            # this applies only to LoRa setting
            self.writeCommand(
                const.RADIO_SET_PACKETPARAMS,
                [
                    packetParam1,
                    packetParam2,
                    packetParam3,
                    packetParam4,
                    packetParam5,
                ],
            )

    def setDioIrqParams(
        self, irqMask: int, dio1Mask: int, dio2Mask: int, dio3Mask: int
    ):
        self.savedIrqMask = irqMask
        self.savedDio1Mask = dio1Mask
        self.savedDio2Mask = dio2Mask
        self.savedDio3Mask = dio3Mask

        self.writeCommand(
            const.RADIO_SET_DIOIRQPARAMS,
            [
                (irqMask >> 8) & 0xFF,
                irqMask & 0xFF,
                (dio1Mask >> 8) & 0xFF,
                dio1Mask & 0xFF,
                (dio2Mask >> 8) & 0xFF,
                dio2Mask & 0xFF,
                (dio3Mask >> 8) & 0xFF,
                dio3Mask & 0xFF,
            ],
        )

    def setSyncWord1(self, syncword):
        logger.debug(f"setSyncWord: {syncword}")

        self.writeRegister( const.REG_FLRCSYNCWORD1_BASEADDR, ( syncword >> 24 ) & 0x000000FF )
        self.writeRegister( const.REG_FLRCSYNCWORD1_BASEADDR + 1, ( syncword >> 16 ) & 0x000000FF )
        self.writeRegister( const.REG_FLRCSYNCWORD1_BASEADDR + 2, ( syncword >> 8 ) & 0x000000FF )
        self.writeRegister( const.REG_FLRCSYNCWORD1_BASEADDR + 3, syncword & 0x000000FF )

    def setHighSensitivity(self):
        self.writeRegister(
            const.REG_LNA_REGIME, (self.readRegister(const.REG_LNA_REGIME) | 0xC0)
        )

    def setLowPowerRX(self):
        self.writeRegister(
            const.REG_LNA_REGIME, (self.readRegister(const.REG_LNA_REGIME) & 0x3F)
        )

    def printModemSettings(self):
        self.printDevice()

        string_builder = "Modem Settings:"

        string_builder += f" PACKET_TYPE_{self.savedPacketType}"
        string_builder += f", FREQ: {self.getFreqInt()}Hz"
        if (
            self.savedPacketType == const.PACKET_TYPE_LORA
            or self.savedPacketType == const.PACKET_TYPE_RANGING
        ):
            string_builder += f", SF: {self.getLoRaSF()}"
            string_builder += f", BW: {self.returnBandwidth(self.savedModParam2)}"
            string_builder += f", CR4: {self.getLoRaCodingRate() + 4}"
        elif self.savedPacketType == const.PACKET_TYPE_FLRC:
            string_builder += f", BandwidthBitRate_{self.savedModParam1}, CodingRate_{self.savedModParam2}, BT_{self.savedModParam3}"

        logger.info(string_builder)

    def printDevice(self):

        if self._device_ID == const.DEVICE_SX1280:
            logger.info("Device: SX1280")
        elif self._device_ID == const.DEVICE_SX1281:
            logger.info("Device: SX1281")
        else:
            logger.error("Device: Unknown")

    def getFreqInt(self):
        msb = 0
        mid = 0
        lsb = 0

        if self.savedPacketType == const.PACKET_TYPE_LORA:
            msb = self.readRegister(const.REG_RFFrequency23_16)
            mid = self.readRegister(const.REG_RFFrequency15_8)
            lsb = self.readRegister(const.REG_RFFrequency7_0)
        elif self.savedPacketType == const.PACKET_TYPE_RANGING:
            msb = self.readRegister(const.REG_RFFrequency23_16)
            mid = self.readRegister(const.REG_RFFrequency15_8)
            lsb = self.readRegister(const.REG_RFFrequency7_0)
        elif self.savedPacketType == const.PACKET_TYPE_FLRC:
            msb = self.readRegister(const.REG_FLRC_RFFrequency23_16)
            mid = self.readRegister(const.REG_FLRC_RFFrequency15_8)
            lsb = self.readRegister(const.REG_FLRC_RFFrequency7_0)

        floattemp = (msb * 0x10000) + (mid * 0x100) + lsb
        floattemp = (floattemp * const.FREQ_STEP) / 1000000
        uinttemp = int(floattemp * 1000000)
        return uinttemp

    def getLoRaSF(self):
        return self.savedModParam1 >> 4

    def returnBandwidth(self, data: int):

        if data == const.LORA_BW_0200:
            return 203125
        elif data == const.LORA_BW_0400:
            return 406250
        elif data == const.LORA_BW_0800:
            return 812500
        elif data == const.LORA_BW_1600:
            return 1625000
        else:
            return 0

    def getLoRaCodingRate(self):
        return self.savedModParam3

    def getInvertIQ(self):
        return self.savedPacketParam5

    def getPreamble(self):
        return self.savedPacketParam1

    def getLNAGain(self):
        return self.readRegister(const.REG_LNA_REGIME) & 0xC0

    def printOperatingSettings(self):
        self.printDevice()

        string_builder = "Operating Settings:"

        string_builder += f" PACKET_TYPE_{self.savedPacketType}"

        if (
            self.savedPacketType == const.PACKET_TYPE_LORA
            or self.savedPacketType == const.PACKET_TYPE_RANGING
        ):
            string_builder += f", Preamble_{self.getPreamble()}"
            
            if self.savedPacketParam2 == const.LORA_PACKET_VARIABLE_LENGTH:
                string_builder += ", Explicit"
            elif self.savedPacketParam2 == const.LORA_PACKET_FIXED_LENGTH:
                string_builder += ", Implicit"
            else:
                string_builder += ", Unknown"

            string_builder += f", PayloadL_{self.savedPacketParam3}, {self.savedPacketParam4}, {self.getInvertIQ()}"
            if self.getLNAGain() == 0xC0:
                string_builder += ", HighSensitivity"
            else:
                string_builder += ", LowPowerRX"
        elif self.savedPacketType == const.PACKET_TYPE_FLRC:
            string_builder += (
                f", Preamble_{((self.savedPacketParam1 >> 4) * 4) + 4}_BITS"
            )
            string_builder += f", SyncWordLength_{self.savedPacketParam2}"
            string_builder += f", SyncWordMatch_{self.savedPacketParam3}"
            string_builder += f", {self.savedPacketParam4}"
            string_builder += f", PayloadLength_{self.savedPacketParam5}, CRC_{self.savedPacketParam6 >> 4}_BYTES, Whitening_{self.savedPacketParam7}"
            if self.getLNAGain() == 0xC0:
                string_builder += ", HighSensitivity"
            else:
                string_builder += ", LowPowerRX"

        logger.info(string_builder)

    def printRegisters(self, start: int, end: int):
        logger.info("Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F")
        logger.info("")

        for i in range(start, end, 16):
            line_buffer = f"{i:04X}  "

            for j in range(16):
                line_buffer += f" {self.readRegister(i + j):02X}"

            logger.info(line_buffer)

    def setPayloadLength(self, length: int):
        if self.savedPacketType == const.PACKET_TYPE_LORA:
            self.writeRegister(const.REG_LR_PAYLOADLENGTH, length)
            self.setPacketParams(
                self.savedPacketParam1,
                self.savedPacketParam2,
                length,
                self.savedPacketParam4,
                self.savedPacketParam5,
            )
        elif self.savedPacketType == const.PACKET_TYPE_FLRC:
            self.writeRegister(const.REG_LR_FLRCPAYLOADLENGTH, length)
            self.setPacketParams(
                self.savedPacketParam1,
                self.savedPacketParam2,
                self.savedPacketParam3,
                self.savedPacketParam4,
                length,
                self.savedPacketParam6,
                self.savedPacketParam7,
            )

    def transmit(self, buffer: str, timeout: int, txPower: int, wait: int):
        if len(buffer) == 0:
            return False

        self.setMode(const.MODE_STDBY_RC)
        self.checkBusy()

        with self._device as spi:
            bts = [const.RADIO_WRITE_BUFFER, 0] + [ord(c) for c in buffer]
            self._spiNSS.low()
            buffer = spi.xfer3(bts)
            self._spiNSS.high()

            buffer = buffer[2:]

            bts_split = [bts[i : i + 8] for i in range(0, len(bts), 8)]
            buffer_split = [buffer[i : i + 8] for i in range(0, len(buffer), 8)]

            idx = 0
            for b in bts_split:
                hex_str = ":".join([f"{c:02X}" for c in b])
                buf_str = ":".join([f"{c:02X}" for c in buffer_split[idx]])
                logger.debug(f"transmit\t {idx}: {hex_str}\t->\t{buf_str}")
                idx += 1

        self._txPacketL = len(buffer)

        self.setPayloadLength(self._txPacketL)
        self.setTxParams(txPower, const.RAMP_TIME)
        self.setDioIrqParams(
            const.IRQ_RADIO_ALL, const.IRQ_TX_DONE + const.IRQ_RX_TX_TIMEOUT, 0, 0
        )
        self.setTx(timeout)

        if not wait:
            return self._txPacketL
        
        while not self._TXDonePin.read():
            print(self._TXDonePin.read())
            time.sleep(.2)
            pass

        self.setMode(const.MODE_STDBY_RC)

        if self.readIrqStatus() & const.IRQ_RX_TX_TIMEOUT:
            return 0
        else:
            return self._txPacketL

    def setTxParams(self, txPower: int, rampTime: int):
        self.savedTXPower = txPower
        self.writeCommand(
            const.RADIO_SET_TXPARAMS,
            [
                (txPower + 18) & 0xFF,
                rampTime & 0xFF,
            ]
        )

    def setTx(self, timeout: int):
        if self._rxtxpinmode:
            self.txEnable()

        self.clearIrqStatus(const.IRQ_RADIO_ALL)

        self.writeCommand(
            const.RADIO_SET_TX,
            [
                const._PERIODBASE,
                (timeout >> 8) & 0x00FF,
                timeout & 0x00FF,
            ],
        )

    def clearIrqStatus(self, irqMask: int):
        self.writeCommand(
            const.RADIO_CLR_IRQSTATUS,
            [
                (irqMask >> 8) & 0xFF,
                irqMask & 0xFF,
            ],
        )

    def readIrqStatus(self):
        buffer = self.readCommand(const.RADIO_GET_IRQSTATUS, 2)
        return (buffer[0] << 8) + buffer[1]

    def receive(self, rxTimeout: int, wait: int):
        logger.debug("receiving started...")

        self.setDioIrqParams(
            const.IRQ_RADIO_ALL,
            (const.IRQ_RX_DONE + const.IRQ_RX_TX_TIMEOUT + const.IRQ_HEADER_ERROR),
            0,
            0,
        )
        logger.debug("setRx")
        self.setRx(rxTimeout)

        if not wait:
            return []
        
        while not self._RXDonePin.read():
            pass

        # stop further packet reception
        self.setMode(const.MODE_STDBY_RC)

        regData = self.readIrqStatus()

        if (
            (regData & const.IRQ_HEADER_ERROR)
            or (regData & const.IRQ_CRC_ERROR)
            or (regData & const.IRQ_RX_TX_TIMEOUT)
            or (regData & const.IRQ_SYNCWORD_ERROR)
        ):
            logger.error(f"Error in packet reception: {regData}")
            return []

        buffer = self.readCommand(const.RADIO_GET_RXBUFFERSTATUS, 2)
        logger.debug(f"Buffer status: {buffer}")

        _rxPacketL = buffer[0]
        rxStart = buffer[1]
        rxEnd = rxStart + _rxPacketL

        logger.debug(f"rxPacketL: {_rxPacketL}, rxStart: {rxStart}, rxEnd: {rxEnd}")

        self.checkBusy()

        with self._device as spi:
            bts = [const.RADIO_READ_BUFFER, rxStart, 0xFF] + [0xFF] * rxEnd
            self._spiNSS.low()
            buffer = spi.xfer3(bts)
            self._spiNSS.high()
            buffer = buffer[3:]

            hex_str = ":".join([f"{b:02X}" for b in bts])
            hex_rec = ":".join([f"{b:02X}" for b in buffer])
            logger.debug(f"receive <> {hex_str} -> {hex_rec}")

        return buffer

    def receiveSXBuffer(self, startAddress: int, rxTimeout: int, wait: int):
        self.setMode(const.MODE_STDBY_RC)
        self.setBufferBaseAddress(0, startAddress)

        self.setDioIrqParams(const.IRQ_RADIO_ALL, (const.IRQ_RX_DONE + const.IRQ_RX_TX_TIMEOUT + const.IRQ_HEADER_ERROR), 0, 0)
        self.setRx(rxTimeout)

        if not wait:
            return 0
        
        while not self._RXDonePin.read():
            pass

        # stop further packet reception
        self.setMode(const.MODE_STDBY_RC)

        regData = self.readIrqStatus()

        if (
            (regData & const.IRQ_HEADER_ERROR)
            or (regData & const.IRQ_CRC_ERROR)
            or (regData & const.IRQ_RX_TX_TIMEOUT)
            or (regData & const.IRQ_SYNCWORD_ERROR)
        ):
            logger.error(f"Error in packet reception: {regData}")
            return 0
        
        buffer = self.readCommand(const.RADIO_GET_RXBUFFERSTATUS, 2)
        logger.debug(f"Buffer status: {buffer}")

        return buffer[0]

    def setRx(self, timeout: int):
        self.clearIrqStatus(const.IRQ_RADIO_ALL)

        if self._rxtxpinmode:
            self.rxEnable()

        self.writeCommand(
            const.RADIO_SET_RX,
            [const._PERIODBASE, (timeout >> 8) & 0x00FF, timeout & 0x00FF],
        )

    def readPacketRSSI(self):
        status = self.readCommand(const.RADIO_GET_PACKETSTATUS, 5)\
        
        rssi = 0
        snr = self.readPacketSNR()

        if self.savedPacketType == const.PACKET_TYPE_LORA:
            rssi = -status[0] / 2
            if snr < 0:
                rssi = rssi + snr
        elif self.savedPacketType == const.PACKET_TYPE_FLRC:
            rssi = -status[0] / 2

        return rssi

    def readPacketSNR(self):
        status = self.readCommand(const.RADIO_GET_PACKETSTATUS, 5)

        if status[1] < 128:
            snr = status[1] / 4
        else:
            snr = (status[1] - 256) / 4

        return snr
