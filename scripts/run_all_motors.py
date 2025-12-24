import can
import time

def main():
	time.sleep(5)
	print("Starting motor test...")
	
	try:
		bus = can.Bus(channel='can0', interface='socketcan')
	except Exception as e:
		print("Error opening CAN bus:", e)
		return
	
	motorIds = [0x144]

	cmdMove = [0xA8, 0x00, 0x32, 0x00, 0xC8, 0x00, 0x00, 0x00]
	cmdStop = [0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

	for i in range(45):
		print(f"Running motor test iteration {i+1}/45")
		for motorId in motorIds:
			print(f"Running motor ID: {hex(motorId)}")
			msgSpeed = can.Message(arbitration_id=motorId, data=cmdMove, is_extended_id=False)
			bus.send(msgSpeed)
		time.sleep(1)

	for motorId in motorIds:
		print(f"Stopping motor ID: {hex(motorId)}")
		msgStop = can.Message(arbitration_id=motorId, data=cmdStop, is_extended_id=False)
		bus.send(msgStop)

	bus.shutdown()

if __name__ == "__main__":
	main()