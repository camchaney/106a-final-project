from time import sleep
import serial
import serial.tools.list_ports
import colorsys
import time

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
		self.send_command("<0,0,0,69>")
		self.status = 0

	def toggle(self):
		if self.status:
			self.off()
		else:
			self.on()

num_lights = 27				# number of lights

lc = LightController()

# Theater
# firstPixelHue = 0
# for a in range(30): 			# repeat 30 times
# 	for b in range(3):
# 		lc.clear();
# 		for c in range(0,num_lights,3):
# 			hue = firstPixelHue + c*65536.0 / num_lights
# 			hue = (hue / 65536) * 179
# 			(r,g,b) = colorsys.hsv_to_rgb(hue,1,150)
# 			#print(hue)
# 			#print(color)
# 			command = f"<{int(r)},{int(g)},{int(b)},{c}>"
# 			print(command)
# 			lc.send_command(command)
# 			time.sleep(0.01)
# 		firstPixelHue += 65536 / 90
# 		time.sleep(0.1)

# Simple run
lc.clear()
d = 6.48			# distance between pixels in mm
sleep = 0.05
for n in range(3):					# do 3 times
	b = 0
	for a in range(30):
		for i in range(1,num_lights):
			if i == b:
				command = f"<0,0,100,{i}>"
			else:
				command = f"<100,100,100,{i}>"
			print(command)
			lc.send_command(command)
			#time.sleep(0.01)
		b += 1
		time.sleep(sleep)
lc.clear()
print(f"Max speed = {d / sleep} mm/s")	# max speed for all light cmds sending at once
			# assuming that we are doing a square grid of pixels


# Rainbow
# highHue = 5*65535
# lc.clear()
# #print(highHue)
# for firstPixelHue in range(0,highHue,256):
# 	print(firstPixelHue)
# 	for i in range(num_lights):
# 		pixelHue = firstPixelHue + (i * 65536 / num_lights)
# 		(r,g,b) = colorsys.hsv_to_rgb(pixelHue,1,150)
# 		command = f"<{int(r)},{int(g)},{int(b)},{i}>"
# 		print(command)
# 		lc.send_command(command)
# 		time.sleep(0.05)					# pixel to pixel delay
# 	time.sleep(0.1)					# system delay (same as example)

