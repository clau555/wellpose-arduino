#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function, division, unicode_literals

import json
import math
import time

import Adafruit_GPIO as GPIO
import Adafruit_GPIO.I2C as I2C
import RPi.GPIO as GPIO_B
import urllib2

################################################################################
# Settings

ACCELERATION_SENSITIVITY = 0.1
AVG_DURATION = 0.5 * 1000
FRAME_DURATION = 5 * 1000
API_URL = "https://wellpose.ythepaut.com"
API_ENDPOINT = "/api/activity"

################################################################################
# Constants

BUZZER_PIN = 16

ACCELERATION_RANGE = 2
ADC_REF = 5.0

ZERO_X = 1.569
ZERO_Y = 1.569
ZERO_Z = 1.569

SENSITIVITY_X = 0.3
SENSITIVITY_Y = 0.3
SENSITIVITY_Z = 0.3


def calcAccel(raw):
	"""
	:type raw: int
	:rtype float
	"""
	return raw * 0.061 * (ACCELERATION_RANGE >> 1) / 1000


def millis():
	"""
	:rtype float
	"""
	return time.time() * 1000


class LSM6DS3:
	def __init__(self, address=0x6a):
		"""
		:type address: int
		"""
		self.i2c = I2C.get_i2c_device(address)
		self.address = address

		dataToWrite = 0
		dataToWrite |= 0x03
		dataToWrite |= 0x00
		dataToWrite |= 0x10
		self.i2c.write8(0X10, dataToWrite)

		self.accel_center_x = self.i2c.readS16(0X28)
		self.accel_center_y = self.i2c.readS16(0x2A)
		self.accel_center_z = self.i2c.readS16(0x2C)

	def readRawAccelX(self):
		"""
		:rtype int
		"""
		return self.i2c.readS16(0X28)

	def readRawAccelY(self):
		"""
		:rtype int
		"""
		return self.i2c.readS16(0x2A)

	def readRawAccelZ(self):
		"""
		:rtype int
		"""
		return self.i2c.readS16(0x2C)

	def readFloatAccelX(self):
		"""
		:rtype float
		"""
		return calcAccel(self.readRawAccelX())

	def readFloatAccelY(self):
		"""
		:rtype float
		"""
		return calcAccel(self.readRawAccelY())

	def readFloatAccelZ(self):
		"""
		:rtype float
		"""
		return calcAccel(self.readRawAccelZ())

	def getXRotation(self):
		"""
		:rtype float
		"""
		value_y = self.readRawAccelY()
		value_z = self.readRawAccelZ()

		yv = (value_y / 1024.0 * ADC_REF - ZERO_Y) / SENSITIVITY_Y
		zv = (value_z / 1024.0 * ADC_REF - ZERO_Z) / SENSITIVITY_Z
		angle_x = math.atan2(-yv, -zv) * 57.2957795 + 180

		return angle_x

	def getYRotation(self):
		"""
		:rtype float
		"""
		value_x = self.readRawAccelX()
		value_z = self.readRawAccelZ()

		xv = (value_x / 1024.0 * ADC_REF - ZERO_X) / SENSITIVITY_X
		zv = (value_z / 1024.0 * ADC_REF - ZERO_Z) / SENSITIVITY_Z
		angle_y = math.atan2(-xv, -zv) * 57.2957795 + 180

		return angle_y

	def getZRotation(self):
		"""
		:rtype float
		"""
		value_x = self.readRawAccelX()
		value_y = self.readRawAccelY()

		xv = (value_x / 1024.0 * ADC_REF - ZERO_X) / SENSITIVITY_X
		yv = (value_y / 1024.0 * ADC_REF - ZERO_Y) / SENSITIVITY_Y
		angle_z = math.atan2(-yv, -xv) * 57.2957795 + 180

		return angle_z


class Vector3:
	def __init__(self, x=0, y=0, z=0):
		"""
		:type x float
		:type y float
		:type z float
		"""
		self.x = x
		self.y = y
		self.z = z

	def norm(self):
		"""
		:rtype float
		"""
		return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)


class Acceleration:
	def __init__(self, v=Vector3(), n=0, t=0):
		"""
		:type v: Vector3
		:type n: int
		:type t: float
		"""
		self.v = v
		self.n = n
		self.t = t

	def toJSON(self):
		"""
		:rtype dict
		"""
		return {
			"acceleration": {
				"x": self.v.x / self.n,
				"y": self.v.y / self.n,
				"z": self.v.z / self.n,
			},
			"dt": self.t
		}


class State:
	"""
	:type gyro: LSM6DS3
	:type accelerations: list[Acceleration]
	:type lastFrameTime: float
	:type lastAvgTime: float
	:type avgMode: bool
	"""

	def __init__(self):
		self.gyro = LSM6DS3()
		self.accelerations = []
		self.lastFrameTime = millis()
		self.lastAvgTime = millis()
		self.avgMode = False


def handleAvgMode(state, acceleration):
	"""
	:type state: State
	:type acceleration: Vector3
	"""
	state.accelerations[-1].v.x += acceleration.x
	state.accelerations[-1].v.y += acceleration.y
	state.accelerations[-1].v.z += acceleration.z
	state.accelerations[-1].n += 1

	if millis() > state.lastAvgTime + AVG_DURATION:
		state.avgMode = False


def beep():
	GPIO_B.output(BUZZER_PIN, GPIO_B.HIGH)
	time.sleep(0.1)
	GPIO_B.output(BUZZER_PIN, 0)


def registerValues(state, acceleration):
	"""
	:type state: State
	:type acceleration: Vector3
	"""
	state.accelerations.append(Acceleration(
		acceleration,
		1,
		millis() - state.lastFrameTime
	))

	state.lastAvgTime = millis()
	state.avgMode = True


def handleResponse(res):
	"""
	:type res: str|None
	"""
	if res is None:
		print("No response")
		return

	res = res.strip()

	try:
		data = json.loads(res)
	except ValueError:
		print("Invalid JSON: " + res)
		return

	if "level" not in data:
		print("No level in response: " + res)
		return

	if data["level"] == 0:
		pass
	elif data["level"] == 1:
		beep()
	elif data["level"] == 2:
		beep()
	else:
		print("Unknown level:", data["level"])


def sendData(state):
	"""
	:type state: State
	"""
	payload = json.dumps({
		"duration": FRAME_DURATION,
		"accelerations": list(
			map(lambda acc: acc.toJSON(), state.accelerations)
		),
		"orientation": {
			"x": state.gyro.getXRotation(),
			"y": state.gyro.getYRotation(),
			"z": state.gyro.getZRotation(),
		}
	}).encode("utf-8")
	print(payload)

	headers = {
		"Content-Type": "application/json"
	}

	req = urllib2.Request(API_URL + API_ENDPOINT, payload, headers)
	res = None
	try:
		con = urllib2.urlopen(req)
		print(con.code, con.msg)
		res = con.read()
		con.close()
	except Exception as e:
		if "read" in dir(e):
			print(e, e.read())
		else:
			print(e)

	handleResponse(res)

	state.accelerations = []
	state.avgMode = False
	state.lastFrameTime = millis()


def loop(state):
	"""
	:type state: State
	"""
	acceleration = Vector3(
		state.gyro.readFloatAccelX(),
		state.gyro.readFloatAccelY(),
		state.gyro.readFloatAccelZ(),
	)

	accNorm = acceleration.norm()

	if state.avgMode:
		handleAvgMode(state, acceleration)
	elif abs(1 - accNorm) > ACCELERATION_SENSITIVITY:
		registerValues(state, acceleration)

	if millis() > state.lastFrameTime + FRAME_DURATION:
		sendData(state)


def main():
	print("start")

	state = None
	while state is None:
		try:
			state = State()
		except IOError:
			print("init error")
			time.sleep(1)

	GPIO_B.setmode(GPIO_B.BCM)
	GPIO_B.setup(BUZZER_PIN, GPIO.OUT)
	GPIO_B.output(BUZZER_PIN, 0)
	beep()

	finished = False
	while not finished:
		try:
			loop(state)
			time.sleep(0.01)
		except KeyboardInterrupt:
			print("end")
			finished = True

	GPIO_B.cleanup()


if __name__ == "__main__":
	main()
