import RPi.GPIO as GPIO
import VL53L1X
import argparse
import time
import numpy as np
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

def auto_int(x):
	return int(x, 0)

class Lidar:
	

	
	def __init__(self,servo_Pin,Xshut_pin,addr,arduinoDevice):
		self.arduino=arduinoDevice
		self.Xshut_pin=Xshut_pin
		self.address_Second_Sensor=addr
		
		GPIO.setmode(GPIO.BCM) #set gpio mode, set xshutpin of one sensor to
		GPIO.setup(Xshut_pin,GPIO.OUT)#LOW to shut it down and change address 
		
		GPIO.output(Xshut_pin,GPIO.LOW)#of the other sensor
		#change address of second sensor to 0x30
		self.__change_Address(0x29,self.address_Second_Sensor)
		
		GPIO.output(Xshut_pin,GPIO.HIGH)#turn back on sensor
		self.tof=VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
		self.tof2=VL53L1X.VL53L1X(i2c_bus=1, i2c_address=self.address_Second_Sensor)
		self.tof.open()
		self.tof2.open()
		
		
		#setup servo motor
		factory=PiGPIOFactory()
		self.servo=Servo(servo_Pin,min_pulse_width=0.5/1000,max_pulse_width=2.5/1000,pin_factory=factory)
		

		
		self.servo_Angle=-90
		self.servo.value=-1/3 #set servo to start postion
		time.sleep(1)
		self.rightPos=True
		
		
	def start(self,distance_Mode,Timing_Budget,Intermeasurement_Time):
		self.tof.set_timing(Intermeasurement_Time, Timing_Budget)
		self.tof2.set_timing(Intermeasurement_Time, Timing_Budget)
		self.tof.start_ranging(distance_Mode)
		self.tof2.start_ranging(distance_Mode)
		
	
	def stop(self):
		self.tof.stop_ranging()
		self.tof2.stop_ranging()
		self.tof.close()
		GPIO.output(self.Xshut_pin,GPIO.LOW)#reset second sensor to default address
		self.tof2.close()
		self.__change_Address(self.address_Second_Sensor,0x29)
		GPIO.output(self.Xshut_pin,GPIO.HIGH)
	
	
	def scan_Around(self,angle,n):
		maxRange=1200 #mm
		self._angleScan=np.empty(0)
		self._distanceScan=np.empty(0)
		
		if self.rightPos:#right to left
			for self.servo_Angle in range(0,n):
				self.servo.value=(angle)/180-1+self.servo_Angle*(angle/n)/180
				x,y,angle_rob=self.getOdom()
				#print(angle)
				distance_mm=self.tof.get_distance()
				distance_mm2=self.tof2.get_distance()
				if distance_mm>maxRange:#rescale to max if distance is greater than the maxRange
					distance_mm=maxRange
				if distance_mm2>maxRange:
					distance_mm2=maxRange
				self._distanceScan=np.append(self._distanceScan,[distance_mm,distance_mm2])
				self._angleScan=np.append(self._angleScan,np.round([self.servo.value*180/2+60+angle_rob,self.servo.value*180/2+120+angle_rob],2))#,self.servo.value*180+150
				self.rightPos=False
			#time.sleep(0.1)
		else:#left to right
			for self.servo_Angle in range(1,n+1):
				self.servo.value=-((angle)/180-1)-self.servo_Angle*(angle/n)/180
				x,y,angle_rob=self.getOdom()
				distance_mm=self.tof.get_distance()
				distance_mm2=self.tof2.get_distance()
				if distance_mm>maxRange:#rescale to max if distance is greater than the maxRange
					distance_mm=maxRange
				if distance_mm2>maxRange:
					distance_mm2=maxRange
				self._distanceScan=np.append(self._distanceScan,[distance_mm,distance_mm2])
				self._angleScan=np.append(self._angleScan,np.round([self.servo.value*180/2+60+angle_rob,self.servo.value*180/2+120+angle_rob],2))
				self.rightPos=True
		return self._angleScan,self._distanceScan	
		
		
	def __change_Address(self,addr_current,New_address):
		tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=addr_current)
		tof.open()
		tof.change_address(New_address)
		tof.close()
		
	
	def getOdom(self):
		try:
			msg="O\n"
			self.arduino.write(msg.encode())
			time.sleep(0.01)
			if self.arduino.in_waiting>0:
				bytesRead=self.arduino.readline()
				odom=bytesRead.decode('utf-8')
				odom=odom.split(";")
				x=int(odom[0])*10# mm
				y=int(odom[1])*10# mm
				yaw=np.rad2deg(int(odom[2])/1000.0)# deg
				return x,y,yaw
			
		except:
			print("error_odom")
			return 0,0,0

if __name__=="__main__":

	test=Lidar(13,0x30)
	test.start(1)
