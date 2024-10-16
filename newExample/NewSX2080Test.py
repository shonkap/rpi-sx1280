import RPi.GPIO as GPIO
import spidev
import time

class SX128x:
    def __init__(self, cs_pin, rst_pin, irq_pin, spi_bus=0, spi_device=0):
        self.cs_pin = cs_pin      # Chip Select (NSS)
        self.rst_pin = rst_pin    # Reset Pin
        self.irq_pin = irq_pin    # Interrupt Request Pin (DIO1)

        # Setup GPIO Pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.cs_pin, GPIO.OUT)
        GPIO.setup(self.rst_pin, GPIO.OUT)
        GPIO.setup(self.irq_pin, GPIO.IN)

        # Setup SPI
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = 500000  # Set SPI speed (adjust as needed)

        self.reset()

    def reset(self):
        """Reset the SX128x module."""
        GPIO.output(self.rst_pin, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(self.rst_pin, GPIO.HIGH)
        time.sleep(0.01)

    def spi_write(self, address, data):
        """Write data to SX128x over SPI."""
        GPIO.output(self.cs_pin, GPIO.LOW)
        self.spi.xfer2([address] + data)
        GPIO.output(self.cs_pin, GPIO.HIGH)

    def spi_read(self, address, length):
        """Read data from SX128x over SPI."""
        GPIO.output(self.cs_pin, GPIO.LOW)
        result = self.spi.xfer2([address] + [0x00] * length)
        GPIO.output(self.cs_pin, GPIO.HIGH)
        return result[1:]  # Skip the first byte (address)

    def begin(self, freq, bw, sf, cr, sync_word, pwr, preamble_length):
        """Initialize the SX128x module for LoRa operation."""
        self.set_frequency(freq)
        self.set_bandwidth(bw)
        self.set_spreading_factor(sf)
        self.set_coding_rate(cr)
        self.set_sync_word(sync_word)
        self.set_preamble_length(preamble_length)
        self.set_output_power(pwr)

        return self.standby()

    def set_frequency(self, freq):
        """Set the frequency of the SX128x module."""
        # Convert frequency to the raw value
        frf = int((freq * (1 << 18)) / 52.0)
        self.spi_write(0x86, [frf >> 16, (frf >> 8) & 0xFF, frf & 0xFF])

    def set_bandwidth(self, bw):
        """Set the bandwidth."""
        if bw == 203.125:
            bw_value = 0x34
        elif bw == 406.25:
            bw_value = 0x26
        elif bw == 812.5:
            bw_value = 0x18
        elif bw == 1625.0:
            bw_value = 0x0A
        else:
            raise ValueError("Invalid bandwidth")
        self.spi_write(0x8B, [bw_value])

    def set_spreading_factor(self, sf):
        """Set the LoRa spreading factor."""
        if sf < 5 or sf > 12:
            raise ValueError("Invalid spreading factor")
        sf_value = (sf - 5) << 4
        self.spi_write(0x8B, [sf_value])

    def set_coding_rate(self, cr):
        """Set the LoRa coding rate."""
        cr_value = cr - 4
        self.spi_write(0x8B, [cr_value])

    def set_sync_word(self, sync_word):
        """Set the LoRa sync word."""
        self.spi_write(0x8C, [sync_word])

    def set_preamble_length(self, length):
        """Set the LoRa preamble length."""
        self.spi_write(0x8C, [length >> 8, length & 0xFF])

    def set_output_power(self, pwr):
        """Set the output power."""
        self.spi_write(0x8E, [pwr])

    def transmit(self, data):
        """Transmit data using SX128x."""
        self.spi_write(0x1A, data)
        self.spi_write(0x83, [0x00])  # Set to TX mode

        # Wait for transmission to complete
        while GPIO.input(self.irq_pin) == GPIO.LOW:
            time.sleep(0.01)

    def receive(self, length):
        """Receive data using SX128x."""
        self.spi_write(0x82, [0x00])  # Set to RX mode

        # Wait for reception to complete
        while GPIO.input(self.irq_pin) == GPIO.LOW:
            time.sleep(0.01)

        # Read the received data
        return self.spi_read(0x1B, length)

    def standby(self):
        """Set the SX128x to standby mode."""
        self.spi_write(0x80, [0x00])

    def sleep(self):
        """Set the SX128x to sleep mode."""
        self.spi_write(0x84, [0x01])

    def get_status(self):
        """Get the status of the SX128x module."""
        return self.spi_read(0xC0, 1)[0]

