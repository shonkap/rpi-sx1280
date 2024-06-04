import time
import RPi.GPIO as GPIO

from const import (
    _FREQ_STEP,
    _PACKET_CRC_MODE_ON,
    _PACKET_HEADER_EXPLICIT,
    _PACKET_IQ_NORMAL,
    _PACKET_TYPE_LORA,
    _PACKET_TYPE_RANGING,
    _RADIO_CLR_IRQSTATUS,
    _RADIO_GET_IRQSTATUS,
    _RADIO_GET_PACKETSTATUS,
    _RADIO_READ_REGISTER,
    _RADIO_SET_BUFFERBASEADDRESS,
    _RADIO_SET_DIOIRQPARAMS,
    _RADIO_SET_MODULATIONPARAMS,
    _RADIO_SET_PACKETPARAMS,
    _RADIO_SET_PACKETTYPE,
    _RADIO_SET_REGULATORMODE,
    _RADIO_SET_RFFREQUENCY,
    _RADIO_SET_RX,
    _RADIO_SET_STANDBY,
    _RADIO_SET_TX,
    _RADIO_SET_TXPARAMS,
    _RADIO_WRITE_BUFFER,
    _RADIO_WRITE_REGISTER,
    _RH_BROADCAST_ADDRESS,
    _irq1Def,
    _irq2Def,
)
from lib.io_pin import IOPin
from lib.spi_device import SpiDevice

_mode_mask = 0xE0
_cmd_stat_mask = 0x1C


class SX1280:
    _status = bytearray(1)
    _status_msg = {"mode": "", "cmd": "", "busy": False}
    _status_mode = {
        0: "N/A",
        1: "N/A",
        2: "STDBY_RC",
        3: "STDBY_XOSC",
        4: "FS",
        5: "Rx",
        6: "Tx",
    }
    _status_cmd = {
        0: "N/A",
        1: "Cmd Successful",
        2: "Data Available",
        3: "Timed-out",
        4: "Cmd Error",
        5: "Failure to Execute Cmd",
        6: "Tx Done",
    }
    _BUFFER = bytearray(10)

    pcktparams = {
        "PreambleLength": 12,
        "HeaderType": _PACKET_HEADER_EXPLICIT,
        "PayloadLength": 0x0F,
        "CrcMode": _PACKET_CRC_MODE_ON,
        "InvertIQ": _PACKET_IQ_NORMAL,
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
        _f = int(float(str(freq) + "E" + "9") / _FREQ_STEP)
        if self._debug:
            print(
                "\t\tSX1280 freq: {:G} GHz ({} PLL steps)".format(
                    float(str(freq) + "E" + "9"), _f
                )
            )
        self._send_command(
            bytes(
                [_RADIO_SET_RFFREQUENCY, (_f >> 16) & 0xFF, (_f >> 8) & 0xFF, _f & 0xFF]
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
        self.node = _RH_BROADCAST_ADDRESS
        self.destination = _RH_BROADCAST_ADDRESS
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
        print("SX1280 has been reset")

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
            print("SX1280 Initialised")

    def set_standby(self, state="STDBY_RC"):
        if self._debug:
            print("SX1280 Setting device to Standby")
        if state == "STDBY_RC":
            self._send_command(bytes([_RADIO_SET_STANDBY, 0x00]), True)
        elif state == "STDBY_XOSC":
            self._send_command(bytes([_RADIO_SET_STANDBY, 0x01]), True)

    def clear_irq_status(self, val=[0xFF, 0xFF]):
        if self._debug:
            print("Clearing IRQ Status")
        self._send_command(bytes([_RADIO_CLR_IRQSTATUS] + val), True)

    def set_regulator_mode(self, mode=0x01):
        if self._debug:
            print("Setting Regulator Mode")
        self._send_command(bytes([_RADIO_SET_REGULATORMODE, mode]), True)

    def set_packet_type(self, packetType=_PACKET_TYPE_LORA):
        self._packetType = packetType
        if packetType == "RANGING":
            self._packetType = _PACKET_TYPE_RANGING
        if self._debug:
            print("Setting Packet Type")
        self._send_command(bytes([_RADIO_SET_PACKETTYPE, self._packetType]), True)
        self.packet_type = packetType

    def get_packet_status(self):
        # See Table 11-63
        self._packet_status = []
        p_stat = self._send_command(
            bytes([_RADIO_GET_PACKETSTATUS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        )
        if self._debug:
            [print(hex(i) + " ", end="") for i in self._BUFFER[:6]]
        self.rssiSync = -1 * int(p_stat[2]) / 2
        self.snr = int(p_stat[3]) / 4
        return p_stat

    def set_modulation_params(self, modParam1=0x70, modParam2=0x26, modParam3=0x01):
        # LoRa: modParam1=SpreadingFactor, modParam2=Bandwidth, modParam3=CodingRate
        # LoRa with SF7, (BW1600=0x0A -> changed to BW400=0x26), CR 4/5
        # Must set PacketType first! - See Table 13-48,49,50
        if self._debug:
            print("Setting Modulation parameters")
        self._send_command(
            bytes([_RADIO_SET_MODULATIONPARAMS, modParam1, modParam2, modParam3])
        )
        if self.packet_type == _PACKET_TYPE_LORA:
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
                print("Invalid Spreading Factor")

    def set_packet_params(self):
        if self._debug:
            print(self.pcktparams)
        self._send_command(
            bytes(
                [
                    _RADIO_SET_PACKETPARAMS,
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
            print("Setting Buffer Base Address")
        self._txBaseAddress = txBaseAddress
        self._rxBaseAddress = rxBaseAddress
        self._send_command(
            bytes([_RADIO_SET_BUFFERBASEADDRESS, txBaseAddress, rxBaseAddress])
        )

    def set_tx_param(self, power=0x1F, rampTime=0xE0):
        # power=13 dBm (0x1F), rampTime=20us (0xE0). See Table 11-47
        # P=-18+power -18+0x1F=13
        if self._debug:
            print("Setting Tx Parameters")
        self._send_command(bytes([_RADIO_SET_TXPARAMS, power, rampTime]))

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
            print("Setting DIO IRQ Parameters")
        self._send_command(
            bytes([_RADIO_SET_DIOIRQPARAMS] + irqMask + dio1Mask + dio2Mask + dio3Mask)
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
                print("\t\ttxen:on, rxen:off")
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
                print("HEADER: {}".format([hex(i) for i in payload]))
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
                    print("\t\ttxen:off, rxen:n/a")
        return txdone

    def write_buffer(self, data):
        # Offset will correspond to txBaseAddress in normal operation.
        _offset = self._txBaseAddress
        _len = len(data)
        assert 0 < _len <= 252
        self._busywait()
        with self._device as device:
            device.writebytes(bytes([_RADIO_WRITE_BUFFER, _offset]) + data)

    def set_tx(self, pBase=0x02, pBaseCount=[0x00, 0x00]):
        # Activate transmit mode with no timeout. Tx mode will stop after first packet sent.
        if self._debug:
            print("Setting Tx")
        # self.clear_Irq_Status([8,7])
        self.clear_irq_status()
        self._send_command(bytes([_RADIO_SET_TX, pBase, pBaseCount[0], pBaseCount[1]]))
        self._listen = False

    def set_rx(self, pBase=0x02, pBaseCount=[0xFF, 0xFF]):
        """
        pBaseCount = 16 bit parameter of how many steps to time-out
        see Table 11-22 for pBase values (0xFFFF=continuous)
        Time-out duration = pBase * periodBaseCount
        """
        if self._debug:
            print("\tSetting Rx")
        # self.clear_Irq_Status([8,7])
        self.clear_irq_status()
        self._send_command(bytes([_RADIO_SET_RX, pBase] + pBaseCount))

    def get_irq_status(self, clear=[0xFF, 0xFF], parse=False, debug=False):
        # if self._debug:
        #     print("Getting IRQ Status")
        _irq1, _irq2 = self._send_command(
            bytes([_RADIO_GET_IRQSTATUS, 0x00, 0x00, 0x00])
        )[2:]

        if parse:
            if self._debug:
                print("IRQ[15:8]:{}, IRQ[7:0]:{}".format(hex(_irq1), hex(_irq2)))  #
            _rslt = []
            for i, j in zip(reversed("{:08b}".format(_irq1)), _irq1Def):  # [15:8]
                if int(i):
                    _rslt.append(j)
            for i, j in zip(reversed("{:08b}".format(_irq2)), _irq2Def):  # [7:0]
                if int(i):
                    _rslt.append(j)
            if self._debug:
                print("IRQ Results: {}".format(_rslt))
            return (_rslt, hex(_irq1), hex(_irq2))

        if clear:
            if clear == True:
                clear = [0xFF, 0xFF]
            self._send_command(
                bytes([_RADIO_CLR_IRQSTATUS] + clear)
            )  # clear IRQ status
        return (_irq1, _irq2)

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
            print("TIMEDOUT on DIOwait")
        if hasattr(self, "timeout_handler"):
            self.timeout_handler()
        return False

    def irq_wait(self, bit):
        _t = time.monotonic() + 4
        while time.monotonic() < _t:
            _irq = self.get_irq_status(clear=False)[1] & 1
            # print(hex(_irq))
            if (_irq >> bit) & 1:
                return True
        if self._debug:
            print("TIMEDOUT on IRQwait")
        if hasattr(self, "timeout_handler"):
            self.timeout_handler()
        return False

    def _writeRegister(self, address1, address2, data):
        if self._debug:
            print("Writing to:", hex(address1), hex(address2))
        self._send_command(bytes([_RADIO_WRITE_REGISTER, address1, address2, data]))

    def _readRegister(self, address1, address2):
        if self._debug:
            print("Reading:", hex(address1), hex(address2))
        self._busywait()
        with self._device as device:
            self._BUFFER = device.xfer(
                bytes([_RADIO_READ_REGISTER, address1, address2])
            )
        if self._debug:
            [print(hex(i), " ", end="") for i in self._BUFFER]
            print("")
        self._busywait()
        return self._BUFFER[1]  # TODO this seems wrong

    def _send_command(self, command, stat=False):
        _size = len(command)
        self._busywait()
        with self._device as device:
            self._BUFFER = device.xfer(command)
        if stat:
            if self._debug:
                print(">\tSendCMD CMD:", [hex(i) for i in command])
                print(">\tSendCMD BUF:", [hex(i) for i in self._BUFFER])
                print(">\t{}".format(self._convert_status(self._BUFFER[0])))
        self._busywait()
        return self._BUFFER[:_size]

    def _convert_status(self, status):
        mode = (status & _mode_mask) >> 5
        cmdstat = (status & _cmd_stat_mask) >> 2
        if mode in self._status_mode:
            self._status_msg["mode"] = self._status_mode[mode]
        if cmdstat in self._status_cmd:
            self._status_msg["cmd"] = self._status_cmd[cmdstat]
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
            print("SX1280 timed out on busywait()")
        self.timeouts += 1
        if self.timeouts > 5:
            self.timeouts = 0
            self.reset_io()
        if hasattr(self, "timeout_handler"):
            self.timeout_handler()
        return False
