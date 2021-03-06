#!/usr/bin/env python

PKG='offloadable_face_recognition'
import sys
import unittest
import cv2
import threading
from offloadable_fr_node import Offloadable_FR_Node
from image_output import Post_Processing as pp
from sensor_msgs.msg import Image

## A sample python unit test
class TestPostProcessing(unittest.TestCase):
    def set_up(self):
        super(TestPostProcessing, self).__init__(*args)
        self.test_image = cv2.imread('test_face.jpeg',0)
        self.features = [(20,20),(25,25),(30,30,(50,50),(75,75),(100,100)]
	self.features_lock = threading.Lock()

        def test_post_processing_graphics_overlay(self):
             pp_img = pp.draw_graphics(self.test_image, None, self.features)
             self.assertTrue(pp_img != self.test_image, "Image has been edited by draw graphics!")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'unit_test_face_detector',TestPostProcessing)
