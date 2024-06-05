import time
import RPi.GPIO as GPIO

from loguru import logger
from rpi_sx1280 import const
from rpi_sx1280.io import IOPin, SpiDevice


class SX1280:
    _status = bytearray(1)
    _status_msg = {"mode": "", "cmd": "", "busy": False}
    _BUFFER = bytearray(10)

    pcktparams = {
        "PreambleLength": 12,
        "HeaderType": const.PACKET_HEADER_EXPLICIT,
        "PayloadLength": 0x0F,
        "CrcMode": const.PACKET_CRC_MODE_ON,
        "InvertIQ": const.PACKET_IQ_NORMAL,
    }

    ranging_params = {"SF": 0xA0, "BW": 0x0A, "CR": 0x01}

    _device: SpiDevice
    _debug = False

    packet_type = 0
    default_dio = False
    txen = False
    rxen = False
    frequency = 2.4
    ranging_calibration = False
    rng_rssi = 0
    retry_counter = 0
    timeouts = 0

    @property
    def packet_info(self):
        return (self._packet_len, self._packet_pointer)

    @property
    def frequency_ghz(self):
        return float(str(self.frequency) + "E" + "9")

    @frequency_ghz.setter
    def frequency_ghz(self, freq):
        """
        0xB89D89 = 12098953 PLL steps = 2.4 GHz
        2.4 GHz  = 12098953*(52000000/(2**18))
        """
        self.frequency = freq
        _f = int(float(str(freq) + "E" + "9") / const.FREQ_STEP)
        if self._debug:
            logger.debug(
                "\t\tSX1280 freq: {:G} GHz ({} PLL steps)".format(
                    float(str(freq) + "E" + "9"), _f
                )
            )
        self._send_command(
            bytes(
                [
                    const.RADIO_SET_RFFREQUENCY,
                    (_f >> 16) & 0xFF,
                    (_f >> 8) & 0xFF,
                    _f & 0xFF,
                ]
            )
        )

    @property
    def packet_status(self):
        self.get_packet_status()
        return (self.rssiSync, self.snr)

    @property
    def listen(self):
        return self._listen

    @listen.setter
    def listen(self, enable):
        if enable:
            if not self._listen:
                if self.rxen:
                    self.txen.value = False
                    self.rxen.value = True
                self.set_rx()
                self._listen = True
        else:
            if self.rxen:
                self.rxen.value = False
            self.set_standby("STDBY_RC")
            self._listen = False

    def __init__(self, spi_id, cs_id, busy_gpio, nreset_gpio, debug=False):
        self._debug = debug

        # initialise pins
        self._reset = IOPin(nreset_gpio, GPIO.OUT, GPIO.HIGH)
        self._busy = IOPin(busy_gpio, GPIO.IN)
        self._cs = IOPin(cs_id, GPIO.OUT, GPIO.HIGH)

        # create a new SPI device
        self._device = SpiDevice(spi_id, cs_id)

        self.reset()
        self._busywait()
        self.retry_counter = 0
        self.timeouts = 0

        # Radio Head (RH) Stuff
        self.ack_delay = None
        self.ack_retries = 5
        self.ack_wait = 0.2
        self.sequence_number = 0

        # RH Header Bytes
        self.node = const.RH_BROADCAST_ADDRESS
        self.destination = const.RH_BROADCAST_ADDRESS
        self.identifier = 0
        self.flags = 0

        # default register configuration
        self.default_config()

    def reset(self):
        """
        Reset the IC using nRST pin, takes <85ms to complete.
        First sets the reset pin LOW then after 50ms sets it HIGH and waits for another 30ms.
        """
        self._reset.low()
        time.sleep(0.05)
        self._reset.high()
        time.sleep(0.03)
        if self._debug:
            logger.debug("SX1280 has been reset")

    def reset_io(self):
        """
        Reset the SX1280 device and also initialise a default config in the end.
        """
        self._reset.low()
        self._busy.read()
        time.sleep(5)
        self._reset.high()
        time.sleep(2)
        self.default_config()

    def default_config(self):
        """
        Initialise the device with a default configuration.
        """
        self.sleeping = False
        self._set_ranging = False
        self._ranging = False
        self._status = 0
        self._autoFS = False
        self._listen = False
        self._range_listening = False

        self.set_standby("STDBY_RC")
        self.clear_irq_status()
        self.set_regulator_mode()
        self.set_packet_type()

        self.frequency_ghz = self.frequency

        self.set_modulation_params()
        self.set_packet_params()
        self.set_buffer_base_address()
        self.set_tx_param()
        self.high_sensitivity_lna(True)
        self.set_dio_irq_params()

        # Set Ranging Filter Size to 200 samples
        # self._writeRegister(0x9,0x1E,20)
        # Disable Advanced Ranging
        self._send_command(bytes([0x9A, 0]))

        # Disable long preamble
        # self._send_command(bytes([0x9B,0]))
        # Set Save Context
        self._send_command(bytes([0xD5]))

        if self._debug:
            logger.debug("SX1280 Initialised")

    def set_standby(self, state="STDBY_RC"):
        if self._debug:
            logger.debug("SX1280 Setting device to Standby")
        if state == "STDBY_RC":
            self._send_command(bytes([const.RADIO_SET_STANDBY, 0x00]), True)
        elif state == "STDBY_XOSC":
            self._send_command(bytes([const.RADIO_SET_STANDBY, 0x01]), True)

    def clear_irq_status(self, val=[0xFF, 0xFF]):
        if self._debug:
            logger.debug("Clearing IRQ Status")
        self._send_command(bytes([const.RADIO_CLR_IRQSTATUS] + val), True)

    def set_regulator_mode(self, mode=0x01):
        if self._debug:
            logger.debug("Setting Regulator Mode")
        self._send_command(bytes([const.RADIO_SET_REGULATORMODE, mode]), True)

    def set_packet_type(self, packetType=const.PACKET_TYPE_LORA):
        self._packetType = packetType
        if packetType == "RANGING":
            self._packetType = const.PACKET_TYPE_RANGING
        if self._debug:
            logger.debug("Setting Packet Type")
        self._send_command(bytes([const.RADIO_SET_PACKETTYPE, self._packetType]), True)
        self.packet_type = packetType

    def get_packet_status(self):
        # See Table 11-63
        self._packet_status = []
        p_stat = self._send_command(
            bytes([const.RADIO_GET_PACKETSTATUS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        )
        if self._debug:
            [logger.debug(hex(i) + " ", end="") for i in self._BUFFER[:6]]
        self.rssiSync = -1 * int(p_stat[2]) / 2
        self.snr = int(p_stat[3]) / 4
        return p_stat

    def set_modulation_params(self, modParam1=0x70, modParam2=0x26, modParam3=0x01):
        # LoRa: modParam1=SpreadingFactor, modParam2=Bandwidth, modParam3=CodingRate
        # LoRa with SF7, (BW1600=0x0A -> changed to BW400=0x26), CR 4/5
        # Must set PacketType first! - See Table 13-48,49,50
        if self._debug:
            logger.debug("Setting Modulation parameters")
        self._send_command(
            bytes([const.RADIO_SET_MODULATIONPARAMS, modParam1, modParam2, modParam3])
        )
        if self.packet_type == const.PACKET_TYPE_LORA:
            self._busywait()
            # If the Spreading Factor selected is SF5 or SF6
            if modParam1 in (0x50, 0x60):
                self._writeRegister(0x09, 0x25, 0x1E)
            # If the Spreading Factor is SF7 or SF-8
            elif modParam1 in (0x70, 0x80):
                self._writeRegister(0x09, 0x25, 0x37)
            # If the Spreading Factor is SF9, SF10, SF11 or SF12
            elif modParam1 in (0x90, 0xA0, 0xB0, 0xC0):
                self._writeRegister(0x09, 0x25, 0x32)
            else:
                logger.debug("Invalid Spreading Factor")

    def set_packet_params(self):
        if self._debug:
            logger.debug(self.pcktparams)
        self._send_command(
            bytes(
                [
                    const.RADIO_SET_PACKETPARAMS,
                    self.pcktparams["PreambleLength"],
                    self.pcktparams["HeaderType"],
                    self.pcktparams["PayloadLength"],
                    self.pcktparams["CrcMode"],
                    self.pcktparams["InvertIQ"],
                    0x00,
                    0x00,
                ]
            )
        )

    def set_buffer_base_address(self, txBaseAddress=0x00, rxBaseAddress=0x00):
        if self._debug:
            logger.debug("Setting Buffer Base Address")
        self._txBaseAddress = txBaseAddress
        self._rxBaseAddress = rxBaseAddress
        self._send_command(
            bytes([const.RADIO_SET_BUFFERBASEADDRESS, txBaseAddress, rxBaseAddress])
        )

    def set_tx_param(self, power=0x1F, rampTime=0xE0):
        # power=13 dBm (0x1F), rampTime=20us (0xE0). See Table 11-47
        # P=-18+power -18+0x1F=13
        if self._debug:
            logger.debug("Setting Tx Parameters")
        self._send_command(bytes([const.RADIO_SET_TXPARAMS, power, rampTime]))

    def high_sensitivity_lna(self, enabled=True):
        _reg = self._readRegister(0x8, 0x91)
        if enabled:
            self._writeRegister(0x8, 0x91, _reg | 0xC0)
        else:
            self._writeRegister(0x8, 0x91, _reg & 0x3F)

    def set_dio_irq_params(
        self,
        irqMask=[0xFF, 0xF3],
        dio1Mask=[0x00, 0x03],
        dio2Mask=[0x00, 0x02],
        dio3Mask=[0x40, 0x20],
    ):
        """
        TxDone IRQ on DIO1, RxDone IRQ on DIO2, HeaderError and RxTxTimeout IRQ on DIO3
        IRQmask (bit[0]=TxDone, bit[1]=RxDone)
            0x43:       0x23
            0100 0011   0010 0011
        DIO1mask
            0000 0000   0000 0001
        DIO2mask
            0000 0000   0000 0010
        """
        if self._debug:
            logger.debug("Setting DIO IRQ Parameters")
        self._send_command(
            bytes(
                [const.RADIO_SET_DIOIRQPARAMS]
                + irqMask
                + dio1Mask
                + dio2Mask
                + dio3Mask
            )
        )

    def send(
        self,
        data,
        pin=None,
        irq=False,
        header=True,
        ID=0,
        target=0,
        action=0,
        keep_listening=False,
    ):
        """Send a string of data using the transmitter.
        You can only send 252 bytes at a time
        (limited by chip's FIFO size and appended headers).
        """
        ba = bytearray()
        ba.extend(map(ord, data))
        return self.send_mod(ba, keep_listening=keep_listening, header=header)

    def send_mod(
        self,
        data,
        *,
        keep_listening=False,
        header=False,
        destination=None,
        node=None,
        identifier=None,
        flags=None,
        debug=False,
    ):
        data_len = len(data)
        assert 0 < data_len <= 252
        if self.txen:
            self.rxen.value = False
            self.txen.value = True
            if self._debug:
                logger.debug("\t\ttxen:on, rxen:off")
        if header:
            payload = bytearray(4)
            if destination is None:  # use attribute
                payload[0] = self.destination
            else:  # use kwarg
                payload[0] = destination
            if node is None:  # use attribute
                payload[1] = self.node
            else:  # use kwarg
                payload[1] = node
            if identifier is None:  # use attribute
                payload[2] = self.identifier
            else:  # use kwarg
                payload[2] = identifier
            if flags is None:  # use attribute
                payload[3] = self.flags
            else:  # use kwarg
                payload[3] = flags
            if self._debug:
                logger.debug("HEADER: {}".format([hex(i) for i in payload]))
            data = payload + data
            data_len += 4
        # Configure Packet Length
        self.pcktparams["PayloadLength"] = data_len
        self.set_packet_params()
        self.write_buffer(data)
        self.set_tx()
        txdone = self.wait_for_irq()
        if keep_listening:
            self.listen = True
        else:
            if self.txen:
                self.txen.value = False
                if self._debug:
                    logger.debug("\t\ttxen:off, rxen:n/a")
        return txdone

    def send_fast(self, data, l):
        self.txen.value = True
        self.pcktparams["PayloadLength"] = l
        self.set_packet_params()
        self._busywait()
        with self._device as device:
            device.writebytes(b"\x1a\x00" + data)
        self.set_tx()
        txdone = self.wait_for_irq()
        self.txen.value = False
        return txdone

    def write_buffer(self, data):
        # Offset will correspond to txBaseAddress in normal operation.
        _offset = self._txBaseAddress
        _len = len(data)
        assert 0 < _len <= 252
        self._busywait()
        with self._device as device:
            device.writebytes(bytes([const.RADIO_WRITE_BUFFER, _offset]) + data)

    def read_buffer(self, offset, payloadLen):
        _payload = bytearray(payloadLen)
        self._busywait()
        with self._device as device:
            _payload = device.xfer(bytes([const.RADIO_READ_BUFFER, offset]))
        return _payload

    def dump_buffer(self, dbuffer):
        self._busywait()
        with self._device as device:
            dbuffer = device.xfer(bytes([const.RADIO_READ_BUFFER, 0, 0]))
        # print('Status:',self._convert_status(self._BIGBUFFER[0]))
        # [print(hex(i),end=',') for i in self._BIGBUFFER[1:]]
        # print('')

    def get_bw(self, _bw):
        if _bw == 0x0A:
            bw_hz = 1625000
        elif _bw == 0x18:
            bw_hz = 812500
        elif _bw == 0x26:
            bw_hz = 406250
        elif _bw == 0x34:
            bw_hz = 203125
        else:
            print("bad BW conversion")
            return 0
        return bw_hz

    def set_tx(self, pBase=0x02, pBaseCount=[0x00, 0x00]):
        # Activate transmit mode with no timeout. Tx mode will stop after first packet sent.
        if self._debug:
            logger.debug("Setting Tx")
        # self.clear_Irq_Status([8,7])
        self.clear_irq_status()
        self._send_command(
            bytes([const.RADIO_SET_TX, pBase, pBaseCount[0], pBaseCount[1]])
        )
        self._listen = False

    def set_rx(self, pBase=0x02, pBaseCount=[0xFF, 0xFF]):
        """
        pBaseCount = 16 bit parameter of how many steps to time-out
        see Table 11-22 for pBase values (0xFFFF=continuous)
        Time-out duration = pBase * periodBaseCount
        """
        if self._debug:
            logger.debug("\tSetting Rx")
        # self.clear_Irq_Status([8,7])
        self.clear_irq_status()
        self._send_command(bytes([const.RADIO_SET_RX, pBase] + pBaseCount))

    def get_irq_status(self, clear=[0xFF, 0xFF], parse=False, debug=False):
        # if self._debug:
        #     logger.debug("Getting IRQ Status")
        _irq1, _irq2 = self._send_command(
            bytes([const.RADIO_GET_IRQSTATUS, 0x00, 0x00, 0x00])
        )[2:]

        if parse:
            if self._debug:
                logger.debug(
                    "IRQ[15:8]:{}, IRQ[7:0]:{}".format(hex(_irq1), hex(_irq2))
                )  #
            _rslt = []
            for i, j in zip(reversed("{:08b}".format(_irq1)), const.IRQ1_DEF):  # [15:8]
                if int(i):
                    _rslt.append(j)
            for i, j in zip(reversed("{:08b}".format(_irq2)), const.IRQ2_DEF):  # [7:0]
                if int(i):
                    _rslt.append(j)
            if self._debug:
                logger.debug("IRQ Results: {}".format(_rslt))
            return (_rslt, hex(_irq1), hex(_irq2))

        if clear:
            if clear == True:
                clear = [0xFF, 0xFF]
            self._send_command(
                bytes([const.RADIO_CLR_IRQSTATUS] + clear)
            )  # clear IRQ status
        return (_irq1, _irq2)

    def clear_range_samples(self):
        # to clear, set bit 5 to 1 then to 0
        _reg = self._readRegister(0x9, 0x23)
        # print('Register 0x923:',hex(_reg))
        _reg |= 1 << 5
        # print('Register 0x923:',hex(_reg))
        self._writeRegister(0x9, 0x23, _reg)
        _reg &= ~(1 << 5)
        # print('Register 0x923:',hex(_reg))
        self._writeRegister(0x9, 0x23, _reg)

    def set_ranging_params(
        self, range_addr=[0x01, 0x02, 0x03, 0x04], master=False, slave=False
    ):
        self.set_standby("STDBY_RC")
        self.clear_range_samples()
        self.set_packet_type("RANGING")
        self.set_modulation_params(
            modParam1=self.ranging_params["SF"],
            modParam2=self.ranging_params["BW"],
            modParam3=self.ranging_params["CR"],
        )
        self.pcktparams["PreambleLength"] = 12
        self.pcktparams["PayloadLength"] = 0
        self.set_packet_params()
        self.frequency_ghz = self.frequency
        self.set_buffer_base_address(txBaseAddress=0x00, rxBaseAddress=0x00)
        self.set_tx_param()  # DEFAULT:power=13dBm,rampTime=20us
        if slave:
            self._rangingRole = 0x00
            # Slave Ranging address
            self._writeRegister(0x9, 0x19, range_addr[0])
            self._writeRegister(0x9, 0x18, range_addr[1])
            self._writeRegister(0x9, 0x17, range_addr[2])
            self._writeRegister(0x9, 0x16, range_addr[3])
            # Ranging address length
            self._writeRegister(0x9, 0x31, 0x3)
            # self.set_dio_irq_params(irqMask=[0x7F,0xF3],dio1Mask=[0x00,0x83],dio2Mask=[0x00,0x03],dio3Mask=[0x40,0x20]) # wrong? RangingSlaveResponseDone,RxDone,TxDone
            # self.set_dio_irq_params(irqMask=[0x7F,0xF3],dio1Mask=[0x9,0x80],dio2Mask=[0x00,0x03],dio3Mask=[0x40,0x20]) # RangingMasterRequestValid,RangingSlaveRequestDiscard,RangingSlaveResponseDone
            self.set_dio_irq_params(
                irqMask=[0x7F, 0xF3],
                dio1Mask=[0x1, 0x80],
                dio2Mask=[0x00, 0x03],
                dio3Mask=[0x40, 0x20],
            )  # RangingSlaveRequestDiscard,RangingSlaveResponseDone
        elif master:
            self._rangingRole = 0x01
            # Master Ranging address
            self._writeRegister(0x9, 0x15, range_addr[0])
            self._writeRegister(0x9, 0x14, range_addr[1])
            self._writeRegister(0x9, 0x13, range_addr[2])
            self._writeRegister(0x9, 0x12, range_addr[3])
            # self.set_dio_irq_params(irqMask=[0x7F,0xF3],dio1Mask=[0x7,0x80],dio2Mask=[0x00,0x01],dio3Mask=[0x00,0x00]) # wrong? RangingMasterTimeout,RangingMasterResultValid,RangingSlaveRequestDiscard,RangingSlaveResponseDone
            self.set_dio_irq_params(
                irqMask=[0x7F, 0xF3],
                dio1Mask=[0x6, 0x00],
                dio2Mask=[0x00, 0x01],
                dio3Mask=[0x00, 0x00],
            )  # RangingMasterTimeout,RangingMasterResultValid

        else:
            logger.debug("Select Master or Slave Only")
            return False

        # Set DIO IRQ Parameters
        self.clear_irq_status()

        if self.ranging_calibration == "custom":
            self.set_ranging_calibration(custom=self.rxtxdelay)
        elif self.ranging_calibration:
            self.set_ranging_calibration(zero=True)
        else:
            # Set Ranging Calibration per Section 3.3 of SemTech AN1200.29
            # TODO set based on modulation params
            # self.set_ranging_calibration(CAL='BW1600',SF=5)
            # self.set_ranging_calibration(CAL='BW1600',SF=9)
            self.set_ranging_calibration(CAL="BW1600", SF=10)

        # Set Ranging Role
        self._send_command(bytes([const.RADIO_SET_RANGING_ROLE, self._rangingRole]))

        self.high_sensitivity_lna(True)

        self._set_ranging = True

    def stop_ranging(self):
        self.set_standby("STDBY_RC")
        self.set_packet_type()
        self.set_packet_params()
        self.high_sensitivity_lna(True)
        self.set_dio_irq_params()
        if self.txen:
            self.txen.value = False
            self.rxen.value = False
        self._set_ranging = False

    def read_range(self, raw=True, raw_bytes=False):
        if not self._ranging:
            logger.debug("Start ranging before attempting to read")
            return

        self.set_standby("STDBY_XOSC")
        # enable LoRa modem clock
        _temp = self._readRegister(0x9, 0x7F) | (1 << 1)
        self._writeRegister(0x9, 0x7F, _temp)
        # Set the ranging type for filtered or raw
        _conf = self._readRegister(0x9, 0x24)
        if raw:
            _conf = (_conf & 0xCF) | 0x0
        else:
            # _conf = (_conf & 0xCF) | 0x10 # averaged
            # _conf = (_conf & 0xCF) | 0x20 # debiased
            _conf = (_conf & 0xCF) | 0x30  # filtered
        # logger.debug('Range Data Type:',hex(_conf))
        self._writeRegister(0x9, 0x24, _conf)

        # Read the 24-bit value
        self._rbuff = bytearray(4)
        self._rbuff = self._readRegisters(0x9, 0x61, 4)  # confirmed working
        self.rng_rssi = -1 * self._rbuff[3] / 2
        # logger.debug('rng_rssi: {}'.format(self.rng_rssi))

        self.set_standby("STDBY_RC")

        if raw_bytes:
            return self._rbuff[:3]

        _val = 0 | (self._rbuff[0] << 16)
        _val |= self._rbuff[1] << 8
        _val |= self._rbuff[2]

        # dist in meters = _val * 150/(2^12 * BW in MHz) = 2scomp / (BW in Hz * 36621.09375)
        _2scomp = (
            self.complement2(_val, 24)
            / self.get_bw(self.ranging_params["BW"])
            * 36621.09375
        )
        if raw:
            return _2scomp
        # averaged, debiased, or filtered results
        return _2scomp * 20.0 / 100.0

    def get_freq_error_indicator(self):
        # Read the 20-bit value (based on StuartsProjects github implementation)
        self.set_standby("STDBY_XOSC")
        efeRaw = self._readRegisters(0x9, 0x54, 3)
        efeRaw[0] = efeRaw[0] & 0x0F  # clear bit 20 which is always set
        self.set_standby()
        return efeRaw

    def set_ranging_calibration(self, CAL="BW1600", SF=5, zero=False, custom=False):
        if zero:
            CAL = 0
        elif custom:
            CAL = custom
        else:
            CAL = 0
        self._writeRegister(0x9, 0x2D, CAL & 0xFF)  # cal[7:0]
        self._writeRegister(0x9, 0x2C, (CAL >> 8) & 0xFF)  # cal[15:8]

    def calc_efe(self, efeRaw):
        efe = 0 | (efeRaw[0] << 16)
        efe |= efeRaw[1] << 8
        efe |= efeRaw[2]
        # efe &= 0x0FFFFF # now performed in get_Freq_Error_Indicator step
        efeHz = (
            1.55
            * self.complement2(efe, 20)
            / (1625000 / self.get_bw(self.ranging_params["BW"]))
        )
        return efeHz

    def complement2(self, num, bitCnt):
        retVal = num
        if retVal >= 2 << (bitCnt - 2):
            retVal -= 2 << (bitCnt - 1)
        return retVal

    def get_packet_status(self):
        # See Table 11-63
        self._packet_status = []
        p_stat = self._send_command(
            bytes([const.RADIO_GET_PACKETSTATUS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        )
        # [logger.debug(hex(i)+' ',end='') for i in self._BUFFER[:6]]
        self.rssiSync = -1 * int(p_stat[2]) / 2
        self.snr = int(p_stat[3]) / 4
        return p_stat

    def get_rx_buffer_status(self):
        self._send_command(bytes([const.RADIO_GET_RXBUFFERSTATUS, 0x00, 0x00, 0x00]))
        return self._BUFFER[:4]

    def receive(self, continuous=True, keep_listening=True):
        if not self._listen:
            self.listen = True
        if continuous:
            self._buf_status = self.get_rx_buffer_status()
            self._packet_len = self._buf_status[2]
            self._packet_pointer = self._buf_status[3]
            if self._packet_len > 0:
                if self._debug:
                    logger.debug(
                        f"Offset: {self._packet_pointer}, Length: {self._packet_len}"
                    )
                packet = self.read_buffer(
                    offset=self._packet_pointer, payloadLen=self._packet_len + 1
                )
                if not keep_listening:
                    self.listen = False
                return packet[1:]

    def receive_mod(
        self,
        *,
        keep_listening=True,
        with_header=False,
        with_ack=False,
        timeout=0.5,
        debug=False,
    ):
        timed_out = False
        if not self.default_dio:
            logger.debug("must set default DIO!")
            return False
        if timeout is not None:
            if not self._listen:
                self.listen = True
            start = time.monotonic()
            timed_out = False
            # Blocking wait for interrupt on DIO
            while not timed_out and not self.default_dio.value:
                if (time.monotonic() - start) >= timeout:
                    timed_out = True
        # Radio has received something!
        packet = None
        # Stop receiving other packets
        self.listen = False
        if not timed_out:
            self._buf_status = self.get_rx_buffer_status()
            self._packet_len = self._buf_status[2]
            self._packet_pointer = self._buf_status[3]
            if self._packet_len > 0:
                packet = self.read_buffer(
                    offset=self._packet_pointer, payloadLen=self._packet_len + 1
                )[1:]
            self.clear_irq_status()
            if self._packet_len > 4:
                if (
                    self.node != const.RH_BROADCAST_ADDRESS
                    and packet[0] != const.RH_BROADCAST_ADDRESS
                    and packet[0] != self.node
                ):
                    if debug:
                        logger.debug("Overheard packet:", packet)
                    packet = None
                # send ACK unless this was an ACK or a broadcast
                elif (
                    with_ack
                    and ((packet[3] & const.RH_FLAGS_ACK) == 0)
                    and (packet[0] != const.RH_BROADCAST_ADDRESS)
                ):
                    # delay before sending Ack to give receiver a chance to get ready
                    if self.ack_delay is not None:
                        time.sleep(self.ack_delay)
                    self.send_mod(
                        b"!",
                        keep_listening=keep_listening,
                        header=True,
                        destination=packet[1],
                        node=packet[0],
                        identifier=packet[2],
                        flags=(packet[3] | const.RH_FLAGS_ACK),
                    )
                    # logger.debug('sband ack to {}'.format(packet[1]))
                if not with_header:  # skip the header if not wanted
                    packet = packet[4:]
        # Listen again if necessary and return the result packet.
        if keep_listening:
            self.listen = True
        return packet

    def get_range(
        self, addr=[0, 0, 0, 0], raw=False, timeout=10, t_resend=3, debug=False, delay=1
    ):
        timed_out = False
        irq = []
        if not self.default_dio:
            logger.debug("must set default DIO!")
            return False
        # sleep a delayed amount to give slave time to configure
        time.sleep(delay)
        self.set_ranging_params(range_addr=addr, master=True)
        self.set_tx(pBase=0x02, pBaseCount=[0xFF, 0xFF])  # reduced pbase to 1ms

        if timeout is not None:
            resend = time.monotonic() + t_resend
            timed_out = time.monotonic() + timeout
            while time.monotonic() < timed_out:
                if self.default_dio.value:
                    irq = self.get_irq_status(clear=True, parse=True)[0]
                    if irq:
                        # logger.debug('m',irq)
                        if debug:
                            logger.debug(irq)
                        if "RngMasterResultValid" in irq:
                            self._ranging = True
                            self.clear_irq_status()
                            return self.read_range(raw_bytes=raw)
                        elif "RngMasterTimeout" in irq:
                            logger.debug("\t\t[master] RngMasterTimeout. Resending...")
                            time.sleep(0.5)
                            self.set_ranging_params(range_addr=addr, master=True)
                            self.set_tx(pBase=0x02, pBaseCount=[0xFF, 0xFF])
                if time.monotonic() > resend:
                    self.set_ranging_params(range_addr=addr, master=True)
                    self.set_tx(pBase=0x02, pBaseCount=[0xFF, 0xFF])
                    logger.debug("\t\t[master] resend timout")
                    resend = time.monotonic() + t_resend
            logger.debug("\t\t[master] timed out")
            self.get_irq_status(clear=[0xFF, 0xFF], parse=False)[0]
            self.set_standby()
            return None

    def receive_range(self, addr=[0, 0, 0, 0], timeout=5, t_resend=3, debug=False):
        timed_out = False
        if not self.default_dio:
            logger.debug("must set default DIO!")
            return False
        self.set_ranging_params(range_addr=addr, slave=True)
        self.set_rx(pBase=0x02, pBaseCount=[0xFF, 0xFF])  # reduced pbase to 1ms

        if timeout is not None:
            resend = time.monotonic() + t_resend
            timed_out = time.monotonic() + timeout
            # Blocking wait for interrupt on DIO
            while time.monotonic() < timed_out:
                if self.default_dio.value:
                    irq = self.get_irq_status(clear=True, parse=True)[0]
                    if irq:
                        # logger.debug('s',irq)
                        if debug:
                            logger.debug(irq)
                        if "RngSlaveResponseDone" in irq:
                            self._ranging = True
                            self.set_standby()
                            self.clear_irq_status()
                            if debug:
                                logger.debug("[range slave] responded to range request")
                            return True
                        elif "RngSlaveReqDiscard" in irq:
                            logger.debug(
                                "\t\t[slave] RngSlaveReqDiscard. Listening again"
                            )
                            self.set_ranging_params(range_addr=addr, slave=True)
                            self.set_rx(pBase=0x02, pBaseCount=[0xFF, 0xFF])
                if time.monotonic() > resend:
                    self.set_ranging_params(range_addr=addr, slave=True)
                    self.set_rx(pBase=0x02, pBaseCount=[0xFF, 0xFF])
                    logger.debug("\t\t[slave] receive timeout")
                    resend = time.monotonic() + t_resend
            logger.debug("SLAVE timed out {}".format(time.monotonic()))
            irq = self.get_irq_status(clear=[0xFF, 0xFF], parse=False)
            logger.debug(irq)
            self.set_standby()
            return False

    def wait_for_irq(self):
        if self.default_dio:
            return self.dio_wait(self.default_dio)
        else:
            return self.irq_wait(bit=1)

    def dio_wait(self, pin):
        _t = time.monotonic() + 3
        while time.monotonic() < _t:
            if pin.value:
                self.clear_irq_status()
                return True
        if self._debug:
            logger.debug("TIMEDOUT on DIOwait")
        if hasattr(self, "timeout_handler"):
            self.timeout_handler()
        return False

    def irq_wait(self, bit):
        _t = time.monotonic() + 4
        while time.monotonic() < _t:
            _irq = self.get_irq_status(clear=False)[1] & 1
            # logger.debug(hex(_irq))
            if (_irq >> bit) & 1:
                return True
        if self._debug:
            logger.debug("TIMEDOUT on IRQwait")
        if hasattr(self, "timeout_handler"):
            self.timeout_handler()
        return False

    def _writeRegister(self, address1, address2, data):
        if self._debug:
            logger.debug(f"Writing to: {hex(address1)}, {hex(address2)}")
        self._send_command(
            bytes([const.RADIO_WRITE_REGISTER, address1, address2, data])
        )

    def _readRegister(self, address1, address2):
        if self._debug:
            logger.debug(f"Reading: {hex(address1)}, {hex(address2)}")
        self._busywait()
        with self._device as device:
            self._BUFFER = device.xfer(
                bytes([const.RADIO_READ_REGISTER, address1, address2])
            )
        if self._debug:
            [logger.debug(hex(i), " ", end="") for i in self._BUFFER]
            logger.debug("")
        self._busywait()
        return self._BUFFER[1]  # TODO this seems wrong

    def _readRegisters(self, address1, address2, _length=1):
        """
        read_reg_cmd,   addr[15:8],     addr[7:0],   NOP,    NOP,    NOP,    NOP,  ...
            status        status         status     status  data1   data2   data3  ...
        """
        _size = _length + 4
        if self._debug:
            logger.debug(f"Reading: {hex(address1), hex(address2)}")
        self._busywait()
        with self._device as device:
            self._BUFFER = device.xfer(
                bytes(
                    [const.RADIO_READ_REGISTER, address1, address2, 0] + [0] * _length
                )
            )
        self._busywait()
        if self._debug:
            logger.debug(
                "Read Regs ({}, {}) _BUFFER: {}".format(
                    hex(address1), hex(address2), [hex(i) for i in self._BUFFER]
                )
            )
        return self._BUFFER[4:_size]

    def _send_command(self, command, stat=False):
        _size = len(command)
        self._busywait()
        with self._device as device:
            self._BUFFER = device.xfer(command)
        if stat:
            if self._debug:
                logger.debug(f">\tSendCMD CMD: {[hex(i) for i in command]}")
                logger.debug(f">\tSendCMD BUF: {[hex(i) for i in self._BUFFER]}")
                logger.debug(f">\t{self._convert_status(self._BUFFER[0])}")
        self._busywait()
        return self._BUFFER[:_size]

    def _convert_status(self, status):
        mode = (status & const.MODE_MASK) >> 5
        cmdstat = (status & const.CMD_STAT_MASK) >> 2
        if mode in const.STATUS_MODE:
            self._status_msg["mode"] = const.STATUS_MODE[mode]
        if cmdstat in const.STATUS_CMD:
            self._status_msg["cmd"] = const.STATUS_CMD[cmdstat]
        self._status_msg["busy"] = not bool(status & 0x1)
        return self._status_msg

    def _busywait(self):
        """
        This function waits for the device's busy pin to become 0.
        If the pin is still busy and is not changing then this will reset the IO of the chip.

        Returns:
            Boolean: TRUE if the device is NOT busy.
        """

        # toggle the chip select pin once
        self._cs.low()
        time.sleep(0.05)
        self._cs.high()
        time.sleep(0.05)

        # wait for busy pin to drop
        _t = time.monotonic() + 3
        while time.monotonic() < _t:
            if self._busy.read() == 0:
                return True
        if self._debug:
            logger.debug("SX1280 timed out on busywait()")
        self.timeouts += 1
        if self.timeouts > 5:
            self.timeouts = 0
            self.reset_io()
        if hasattr(self, "timeout_handler"):
            self.timeout_handler()
        return False
