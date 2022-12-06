from time import sleep
import serial
import serial.tools.list_ports

class LightController(object):

	def __init__(self):
		self.port = self.get_serial()
		print(f"connected to serial port: {self.port}")
		self.status = 0

	def get_serial(self):
		try:
			print(serial.tools.list_ports.comports()[-1].device)
			return serial.Serial(serial.tools.list_ports.comports()[-1].device, 115200)
			# return serial.Serial('/dev/ttyUSB0', 9600)
		except:
			# this one is irrelevant
			return serial.Serial('/dev/ttyACM1', 115200)

	def send_command(self, com):
		# Send packet in form: <R,G,B,i>
		#   - R = red value from (0,255)
		#   - G = green value from (0,255)
		#   - B = blue value from (0,255)
		#   - i = index of light to actuate
		try:
			self.port.write(com.encode('utf-8'))
		except:
			print("[error] serial failed to write.")
			self.port.close()
			sleep(5)
		
	def on(self):
		# defaulting to red tip light for now
		self.send_command("<255,0,0,0>")
		self.status = 1

	def off(self):
		self.send_command("<0,0,0,0>")
		self.status = 0

	def clear(self):
		# turn off all LEDs
		self.send_command("<0,0,0,69>")
		self.status = 0

	def toggle(self):
		if self.status:
			self.off()
		else:
			self.on()

	def on_color(self, b, g, r, i):
		self.send_command(f"<{r},{g},{b},{i}>")
		self.status = 1
