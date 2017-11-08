from __future__ import print_function
from btle import UUID, Peripheral, ADDR_TYPE_RANDOM, DefaultDelegate

"""Bluetooth Low Energy Python interface"""
import sys
import os
import time
import subprocess
import binascii
import select
import struct
import argparse

script_path = os.path.join(os.path.abspath(os.path.dirname(__file__)))
helperExe = os.path.join(script_path, "bluepy-helper")

def write_uint16(data, value, index):
	""" Write 16bit value into data string at index and return new string """
	data = data.decode('utf-8')	 # This line is added to make sure both Python 2 and 3 works
	return '{}{:02x}{:02x}{}'.format(
				data[:index*4], 
				value & 0xFF, value >> 8, 
				data[index*4 + 4:])

def write_uint8(data, value, index):
	""" Write 8bit value into data string at index and return new string """
	data = data.decode('utf-8')	 # This line is added to make sure both Python 2 and 3 works
	return '{}{:02x}{}'.format(
				data[:index*2], 
				value, 
				data[index*2 + 2:])

# Please see # Ref https://nordicsemiconductor.github.io/Nordic-Thingy52-FW/documentation
# for more information on the UUIDs of the Services and Characteristics that are being used
def Nordic_UUID(val):
	""" Adds base UUID and inserts value to return Nordic UUID """
	return UUID("EF68%04X-9B35-4933-9B10-52FFA9740042" % val)


# Definition of all UUID used by Thingy
#UART_SERVICE_UUID = 6e400001-b5a3-f393-e0a9-e50e24dcca9e

TEMPERATURE_SERVICE_UUID = 0x1809 # 0x0201
TEMPERATURE_VALUE_UUID = 0x2A1C
TEMPERATURE_CCCD_UUID = 0x2902

UART_SERVICE_UUID = 0x0001

#currently unusedselec
ENVIRONMENT_SERVICE_UUID = 0x0200
E_TEMPERATURE_CHAR_UUID = 0x0201
E_PRESSURE_CHAR_UUID	= 0x0202
E_HUMIDITY_CHAR_UUID	= 0x0203
E_CONFIG_CHAR_UUID		= 0x0206


class UARTService():		
	"""
	UART service module. Instance the class and enable to get access to the UART interface.
	"""
	svcUUID = UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e")
	
	def __init__(self, periph):
		self.periph = periph
		self.service = None
		self.data = None

	def enable(self):
		""" Enables the class by finding the service and its characteristics. """
		if self.service is None:
			self.service = self.periph.getServiceByUUID(self.svcUUID)

	def read(self):
		""" Returns the battery level in percent """
		val = ord(self.data.read())
		return val
		

class TemperatureSensor():
	"""
	Temperature Service module. Instance the class and enable to get access to Temperature interface.
	"""
	svcUUID = UUID(TEMPERATURE_SERVICE_UUID)  # Ref https://www.bluetooth.com/specifications/gatt/services 
	dataUUID = UUID(TEMPERATURE_VALUE_UUID) # Ref https://www.bluetooth.com/specifications/gatt/characteristics
	cccdUUID = UUID(TEMPERATURE_CCCD_UUID) 

	def __init__(self, periph):
		self.periph = periph
		self.service = None
		self.data = None
		self.cccd = None
	def enable(self):
		""" Enables the class by finding the service and its characteristics. """
		if self.service is None:
			self.service = self.periph.getServiceByUUID(self.svcUUID)
		if self.data is None:
			self.data = self.service.getCharacteristics(self.dataUUID)[0]
		if self.cccd is None:
			self.cccd = self.data.getDescriptors(self.cccdUUID)[0]
	def set_notification(self, state):
	
	
	
		print(repr(self.cccd))
		print(self.cccd.uuid.getCommonName())
		print(self.cccd.uuid)
		if self.cccd is not None:
			if state == True:
				#dev.setDelegate(MyDelegate(self.data.getHandle()));
				self.cccd.write("\x02\x00", True)
			else:
				self.cccd.write(b"\x00\x00", True)
	def read(self):
		""" Returns the battery level in percent """
		if self.cccd is not None:
			val = self.cccd.read()
			return val



class EnvironmentService():
	"""
	Environment service module. Instance the class and enable to get access to the Environment interface.
	"""
	serviceUUID =			Nordic_UUID(ENVIRONMENT_SERVICE_UUID)
	temperature_char_uuid = Nordic_UUID(E_TEMPERATURE_CHAR_UUID)
	pressure_char_uuid =	Nordic_UUID(E_PRESSURE_CHAR_UUID)
	humidity_char_uuid =	Nordic_UUID(E_HUMIDITY_CHAR_UUID)
	config_char_uuid =		Nordic_UUID(E_CONFIG_CHAR_UUID)

	def __init__(self, periph):
		self.periph = periph
		self.environment_service = None
		self.temperature_char = None
		self.temperature_cccd = None
		self.pressure_char = None
		self.pressure_cccd = None
		self.humidity_char = None
		self.humidity_cccd = None
		self.config_char = None

	def enable(self):
		""" Enables the class by finding the service and its characteristics. """
		global e_temperature_handle
		global e_pressure_handle
		global e_humidity_handle

		if self.environment_service is None:
			self.environment_service = self.periph.getServiceByUUID(self.serviceUUID)
		if self.temperature_char is None:
			self.temperature_char = self.environment_service.getCharacteristics(self.temperature_char_uuid)[0]
			e_temperature_handle = self.temperature_char.getHandle()
			self.temperature_cccd = self.temperature_char.getDescriptors(forUUID=CCCD_UUID)[0]
		if self.pressure_char is None:
			self.pressure_char = self.environment_service.getCharacteristics(self.pressure_char_uuid)[0]
			e_pressure_handle = self.pressure_char.getHandle()
			self.pressure_cccd = self.pressure_char.getDescriptors(forUUID=CCCD_UUID)[0]
		if self.humidity_char is None:
			self.humidity_char = self.environment_service.getCharacteristics(self.humidity_char_uuid)[0]
			e_humidity_handle = self.humidity_char.getHandle()
			self.humidity_cccd = self.humidity_char.getDescriptors(forUUID=CCCD_UUID)[0]
		if self.config_char is None:
			self.config_char = self.environment_service.getCharacteristics(self.config_char_uuid)[0]

	def set_temperature_notification(self, state):
		if self.temperature_cccd is not None:
			if state == True:
				self.temperature_cccd.write(b"\x01\x00", True)
			else:
				self.temperature_cccd.write(b"\x00\x00", True)

	def set_pressure_notification(self, state):
		if self.pressure_cccd is not None:
			if state == True:
				self.pressure_cccd.write(b"\x01\x00", True)
			else:
				self.pressure_cccd.write(b"\x00\x00", True)

	def set_humidity_notification(self, state):
		if self.humidity_cccd is not None:
			if state == True:
				self.humidity_cccd.write(b"\x01\x00", True)
			else:
				self.humidity_cccd.write(b"\x00\x00", True)

	def configure(self, temp_int=None, press_int=None, humid_int=None, gas_mode_int=None,
						color_int=None, color_sens_calib=None):
		if temp_int is not None and self.config_char is not None:
			current_config = binascii.b2a_hex(self.config_char.read())
			new_config = write_uint16(current_config, temp_int, 0)
			self.config_char.write(binascii.a2b_hex(new_config), True)
		if press_int is not None and self.config_char is not None:
			current_config = binascii.b2a_hex(self.config_char.read())
			new_config = write_uint16(current_config, press_int, 1)
			self.config_char.write(binascii.a2b_hex(new_config), True)
		if humid_int is not None and self.config_char is not None:
			current_config = binascii.b2a_hex(self.config_char.read())
			new_config = write_uint16(current_config, humid_int, 2)
			self.config_char.write(binascii.a2b_hex(new_config), True)

	def disable(self):
		set_temperature_notification(False)
		set_pressure_notification(False)
		set_humidity_notification(False)

class MyDelegate(DefaultDelegate):
	
	def handleNotification(self, hnd, data):
		#print(repr(data)) #debug
		if (hnd == e_temperature_handle):
			teptep = binascii.b2a_hex(data)
			print('Notification: Temp received:	 {}.{} degCelcius'.format(
						self._str_to_int(teptep[:-2]), int(teptep[-2:], 16)))
			
		elif (hnd == e_pressure_handle):
			pressure_int, pressure_dec = self._extract_pressure_data(data)
			print('Notification: Press received: {}.{} hPa'.format(
						pressure_int, pressure_dec))

		elif (hnd == e_humidity_handle):
			teptep = binascii.b2a_hex(data)
			print('Notification: Humidity received: {} %'.format(self._str_to_int(teptep)))

		elif (hnd == m_tap_handle):
			direction, count = self._extract_tap_data(data)
			print('Notification: Tap: direction: {}, count: {}'.format(direction, self._str_to_int(count)))

		elif (hnd == m_orient_handle):
			teptep = binascii.b2a_hex(data)
			print('Notification: Orient: {}'.format(teptep))

		elif (hnd == m_quaternion_handle):
			teptep = binascii.b2a_hex(data)
			print('Notification: Quaternion: {}'.format(teptep))

		elif (hnd == m_stepcnt_handle):
			teptep = binascii.b2a_hex(data)
			print('Notification: Step Count: {}'.format(teptep))

		elif (hnd == m_rawdata_handle):
			teptep = binascii.b2a_hex(data)
			print('Notification: Raw data: {}'.format(teptep))

		elif (hnd == m_euler_handle):
			teptep = binascii.b2a_hex(data)
			print('Notification: Euler: {}'.format(teptep))

		elif (hnd == m_rotation_handle):
			teptep = binascii.b2a_hex(data)
			print('Notification: Rotation matrix: {}'.format(teptep))

		elif (hnd == m_heading_handle):
			teptep = binascii.b2a_hex(data)
			print('Notification: Heading: {}'.format(teptep))

		elif (hnd == m_gravity_handle):
			teptep = binascii.b2a_hex(data)
			print('Notification: Gravity: {}'.format(teptep))		 

		else:
			teptep = binascii.b2a_hex(data)
			print('Notification: UNKOWN: hnd {}, data {}'.format(hnd, teptep))
			

	def _str_to_int(self, s):
		""" Transform hex str into int. """
		i = int(s, 16)
		if i >= 2**7:
			i -= 2**8
		return i	

	def _extract_pressure_data(self, data):
		""" Extract pressure data from data string. """
		teptep = binascii.b2a_hex(data)
		pressure_int = 0
		for i in range(0, 4):
				pressure_int += (int(teptep[i*2:(i*2)+2], 16) << 8*i)
		pressure_dec = int(teptep[-2:], 16)
		return (pressure_int, pressure_dec)
		
	def _extract_tap_data(self, data):
		""" Extract tap data from data string. """
		teptep = binascii.b2a_hex(data)
		direction = teptep[0:2]
		count = teptep[2:4]
		return (direction, count)

class BLE_Device(Peripheral):
	"""
	BLE_Device module. Instance the class and enable to get access to the BLE_Device Sensors.
	The addr of your device has to be know, or can be found by using the hcitool command line 
	tool, for example. Call "> sudo hcitool lescan" and your Thingy's address should show up.
	"""
	def __init__(self, addr):
		Peripheral.__init__(self, addr, addrType=ADDR_TYPE_RANDOM)
		
		# Configuration service not implemented
		self.temperature = TemperatureSensor(self)
		#self.environment = EnvironmentService(self)
		self.uart = UARTService(self)
		#self.ui = UserInterfaceService(self)
		#self.motion = MotionService(self)
		#self.sound = SoundService(self)
		# DFU Service not implemented

def capitaliseName(descr):
	words = descr.replace("("," ").replace(")"," ").replace('-',' ').split(" ")
	capWords =	[ words[0].lower() ]
	capWords += [ w[0:1].upper() + w[1:].lower() for w in words[1:] ]
	return "".join(capWords)

class _UUIDNameMap:
	# Constructor sets self.currentTimeService, self.txPower, and so on
	# from names.
	def __init__(self, idList):
		self.idMap = {}

		for uuid in idList:
			attrName = capitaliseName(uuid.commonName)
			vars(self) [attrName] = uuid
			self.idMap[uuid] = uuid

	def getCommonName(self, uuid):
		if uuid in self.idMap:
			return self.idMap[uuid].commonName
		return None

def get_json_uuid():
	import json
	with open(os.path.join(script_path, 'uuids.json'),"rb") as fp:
		uuid_data = json.loads(fp.read().decode("utf-8"))
	for k in uuid_data.keys():
		for number,cname,name in uuid_data[k]:
			yield UUID(number, cname)
			yield UUID(number, name)

AssignedNumbers = _UUIDNameMap( get_json_uuid() )

def main():
	parser = argparse.ArgumentParser()
	parser.add_argument('-t',action='store',type=float, default=2.0, help='time between polling')
	parser.add_argument('--temperature', action="store_true",default=False)
	parser.add_argument('--uart', action="store_true",default=False)
	args = parser.parse_args()

	print('Connecting...')
	#print('Connecting to ' + args.mac_address)
	dev = BLE_Device("CA:E9:ED:C7:62:98")
	dev._mgmtCmd("le on")
	dev._mgmtCmd("pairable off")
	dev.pair("CA:E9:ED:C7:62:98")
	
	print('Connected...')
	dev.setDelegate(MyDelegate())
 
	try:
		UART_UUID = UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e")
	 
		#UARTService = dev.getServiceByUUID(UART_UUID)
		#for ch in UARTService.getCharacteristics():
		#	print ('getChar')
		#	print str(ch)

		if 0:
			for svc in dev.services:
				print(str(svc), ":")
				for ch in svc.getCharacteristics():
					print("	   {}, hnd={}, supports {}".format(ch, hex(ch.handle), ch.propertiesToString()))
					#chName = AssignedNumbers.getCommonName(ch.uuid)
					if (ch.supportsRead()):
						try:
							print("	   ->", repr(ch.read()))
						except BTLEException as e:
							print("	   ->", e)
							
					for des in svc.getDescriptors():
						print("	   {}, hnd={}".format(des, hex(des.handle)))
						#desName = AssignedNumbers.getCommonName(des.uuid)
		# Enabling selected sensors
		print('Enabling selected sensors...')
		
		
		
		# Environment Service
		if args.temperature:
			dev.temperature.enable()
			print('reading descriptor...')
			print("temp: ", repr(dev.temperature.read()))
			print('writing descriptor...')
			#dev.temperature.set_notification(True)
		""" Enables the class by finding the service and its characteristics. """
		print("temp: ", repr(dev.temperature.read()))
		
		
		print('Getting into loop...')
			
		
		time.sleep(1.0)
		counter=1
		while True:
			
			if counter >= 5:
				break
			
			counter += 1
			#dev.waitForNotifications(args.t)
		
		dev.temperature.set_notification(True)
		#uuidValue = UUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e")
		#lightSensorValue = UARTService.getCharacteristics(uuidValue)[0]
		# Read the sensor
		#val = lightSensorValue.read()
		#print "Light sensor raw value", binascii.b2a_hex(val)
		# Allow sensors time to start up (might need more time for some sensors to be ready)
		#print('All requested sensors and notifications are enabled...')
		#time.sleep(1.0)

	finally:
		dev.disconnect()


if __name__ == "__main__":
	main()
