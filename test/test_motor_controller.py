#!/usr/bin/env python

PKG = 'rospy_tutorials'
NAME = 'peer_subscribe_notify_test'

import sys 
import time
import unittest

import rospy
import rostest

class TestMotorController(unitttest.TestCase):
	pass

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestPeerSubscribeListener, sys.argv)