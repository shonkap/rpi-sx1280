import sys
import select
import time
import subprocess

from lora import LoRa, ModemConfig


def on_recv(message):
	print("From:",message.header_from)
	print("Message:",message.message)


try:
	print(ModemConfig.Bw125Cr45Sf128)
	newlora = LoRa(1, 5, 2, modem_config=ModemConfig.Bw125Cr45Sf128, tx_power=14, acks=True, receive_all=False)

	newlora.on_recv = on_recv
except Exception as e:
	print(e)
	exit()

newlora.set_mode_rx()

message = "Hello there!"
status = newlora.send_to_wait(message, 5, retries=2) #255 is all
if status is False:
		print("No acknowledge from recipient")

print("Press Enter to continue...");
i = 0
while True:

		if select.select([sys.stdin],[],[], 0.1)[0]:
			lineData = sys.stdin.readline().strip()
			if lineData.lower() == "quit":
				break
			newlora.send_to_wait(lineData,5,retries=2)
			if status is False:
					print("No acknowledge from recipient")
			print(lineData)
newlora.close()

