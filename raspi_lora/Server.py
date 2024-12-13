from raspi_lora import LoRa, ModemConfig
import sys
import select
import time
import subprocess
import RPi.GPIO as GPIO

loraInterrupt = 5

GPIO.setmode(GPIO.BCM)
GPIO.setup(loraInterrupt, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def on_recv(message):
	print("From:",message.header_from)
	print("Message:",message.message)

#GPIO.output(25,0)
#time.sleep(2)

try:
	lora = LoRa(1, 5, 2, modem_config=ModemConfig.Bw125Cr45Sf128, tx_power=14, acks=True, receive_all=True)
	GPIO.add_event_detect(loraInterrupt, GPIO.RISING, callback=lora._handle_interrupt)	
	lora.on_recv = on_recv
except Exception as e:
	print(e)
	GPIO.cleanup()
	exit()

lora.set_mode_rx()

message = "Hello there!"
status = lora.send_to_wait(message, 255, retries=2) #255 is all
if status is True:
		print("Message Sent!")
else:
		print("No acknowledge from recipient")

print("Press Enter to continue...");
i = 0
while True:
		#lora._handle_interrupt(1)
		#if not (lora._spi_read(0x13) == 0):
				#lora._handle_interrupt(1)
				#print(lora._spi_read(0x12))
				#print("stuff")
				#break

		if select.select([sys.stdin],[],[], 0.1)[0]:
			lineData = sys.stdin.readline().strip()
			if lineData.lower() == "quit":
				break
			lora.send_to_wait(lineData,255,retries=2)
			print(lineData)

GPIO.cleanup()
