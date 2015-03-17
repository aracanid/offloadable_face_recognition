#!/usr/bin/env python

class MotorError(Exception):
	pass

class CoordinatesTimeOut(MotorError):
	pass
