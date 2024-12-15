import sys
import select
import time
import subprocess
import threading
import time

from lora import LoRa, ModemConfig
clientID = 5
isSatellite = True

def start_shell():
    """Start a persistent shell subprocess."""
    shell = subprocess.Popen(
        "/bin/bash",   # Use the shell you want (e.g., /bin/bash or /bin/sh)
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )
    return shell

def read_shell_output(shell, output_buffer, new_output_event):
    """Continuously read the shell output."""
    while True:
        output = shell.stdout.readline()
        if output:
            output_buffer.append(output)
            new_output_event.set()  # Signal that new output is available
        if shell.poll() is not None:  # Check if the shell has exited
            break

def process_recv(message, headerId):
	global newlora
	global clientID
	
	if isinstance(message,bytes):
		cmd = message.decode("utf-8")
		if cmd[0:3].lower() == "cmd":
			shell.stdin.write(cmd[3:] + "\n")
			shell.stdin.flush()
				
def send_helper(message, lora_to):
	global newlora
	maxlen = 240
	i = 0
	while len(message[i: i+maxlen]) > 1:
		newlora.send(message[i:i+maxlen].encode('utf-8'),lora_to)
		i = i + maxlen
		print(i)
		time.sleep(.1)
	newlora.set_mode_rx()

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

	if isSatellite:
		# Start the persistent shell
		shell = start_shell()
		output_buffer = []
		new_output_event = threading.Event()
	
		# Start a thread to read shell output
		output_thread = threading.Thread(
	        	target=read_shell_output, args=(shell, output_buffer, new_output_event)
	    	)
		output_thread.daemon = True
		output_thread.start()
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

		# Check if there is new output to send
		if new_output_event.is_set() and isSatellite:
			# Collect new output and clear the buffer
			response = ''.join(output_buffer).strip()
			output_buffer.clear()  # Clear the buffer after reading
			new_output_event.clear()  # Reset the event
			print(f"Sending response: {response}")
			send_helper(response, clientID)
		time.sleep(0.1)  # Small delay to reduce CPU usage

shell.terminate()
newlora.close()

