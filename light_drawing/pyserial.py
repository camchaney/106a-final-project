from time import sleep
import serial

# l - left
# r - right
# d - drive
# s - stop

def get_serial():
	try:
		return serial.Serial('/dev/ttyACM0', 9600)
	except:
		return serial.Serial('/dev/ttyACM1', 9600)

def send_command(ser, com):
	try:
		ser.write(com.encode('utf-8'))
		return ser
	except:
		print("[error] serial failed to write.")
		ser.close()
		sleep(5)
		return send_command(get_serial(), com)