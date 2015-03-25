#!/usr/bin/env python

PKG='offloadable_face_recognition'
import sys
import unittest
import cv2
from offloadable_fr_node import Offloadable_FR_Node
from motor_controller import Motor_Controller as mc
from sensor_msgs.msg import Image

## A sample python unit test
class TestMotorController(unittest.TestCase):
    def set_up(self):
        super(TestMotorController, self).__init__(*args)
        self.features = [(20,20),(25,25),(30,30,(50,50),(75,75),(100,100)]
        self.center_x = 50
        self.center_y = 50

    def test_get_features_center(self):
        x, y = cmc.get_features_center(self.features)
        self.assertTrue(x == self.center_x and y == self.center_y, "Center of face is (%d,%d)"%(x,y))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'unit_test_face_detector', TestMotorController)