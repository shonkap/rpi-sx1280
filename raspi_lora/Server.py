import sys
import select
import time
import subprocess

from lora import LoRa, ModemConfig
clientID = 5
isSatellite = True

def process_recv(message, headerId):
	global clientID
	cmd = message[1:]
	if cmd[0:3].lower() == "cmd":
		retOutput = subprocess.getstatusoutput(cmd[3:])
		newlora.send_to_wait(retOutput,clientID,retries=2)
	

def on_recv(message):
	print("From:",message.header_from)
	print("Message:",message.message)
	
	global isSatellite
	if isSatellite:
		process_recv(message.message, message.header_id)

try:
	print(ModemConfig.Bw125Cr45Sf128)
	newlora = LoRa(1, 5, 2, modem_config=ModemConfig.Bw125Cr45Sf128, tx_power=14, acks=True, receive_all=True)

	newlora.on_recv = on_recv
except Exception as e:
	print(e)
	exit()

newlora.set_mode_rx()

message = "Client 2 Online!"
status = newlora.send_to_wait(message, clientID, retries=2) #255 is all
if status is False:
		print("No acknowledge from recipient")

print(	"########################################\n",
	"########################################\n","quit to exit\n",
     	"########################################");
i = 0
while True:
		if select.select([sys.stdin],[],[], 0.1)[0]:
			lineData = sys.stdin.readline().strip()
			if lineData.lower() == "quit":
				break
			newlora.send_to_wait(lineData,clientID,retries=2)
			if status is False:
					print("No acknowledge from recipient")
newlora.close()

