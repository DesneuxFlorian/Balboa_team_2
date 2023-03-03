import RPi.GPIO as GPIO
import VL53L1X
import argparse
import time
import numpy as np

def auto_int(x):
	return int(x, 0)

class Lidar:
	

	
	def __init__(self,Xshut_pin,addr):
		
		self.Xshut_pin=Xshut_pin
		self.address_Second_Sensor=addr
		
		GPIO.setmode(GPIO.BOARD) #set gpio mode, set xshutpin of one sensor to
		GPIO.setup(Xshut_pin,GPIO.OUT)#LOW to shut it down and change address 
		GPIO.output(Xshut_pin,GPIO.LOW)#of the other sensor
		#change address of second sensor to 0x30
		self.__change_Address(0x29,self.address_Second_Sensor)
		
		GPIO.output(Xshut_pin,GPIO.HIGH)#turn back on sensor
		self.tof=VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
		self.tof2=VL53L1X.VL53L1X(i2c_bus=1, i2c_address=self.address_Second_Sensor)
		self.tof.open()
		self.tof2.open()
		
		
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
		
		
	def scan_Around(self):
		distance_mm=self.tof.get_distance()
		distance_mm2=self.tof2.get_distance()
		return distance_mm,distance_mm2
		
	def __change_Address(self,addr_current,New_address):
		tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=addr_current)
		tof.open()
		tof.change_address(New_address)
		tof.close()

if __name__=="__main__":

	test=Lidar(13,0x30)
	test.start(1)
