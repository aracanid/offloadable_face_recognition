#!/usr/bin/env python

PKG='offloadable_face_recognition'
import sys
import unittest
import cv2
from image_pre_processing import Image_Pre_Processing as ipp
from sensor_msgs.msg import Image
import numpy as np

## A sample python unit test
class TestPreProcessing(unittest.TestCase):

	def set_up(self):
		self.test_image = cv2.imread('test_face.jpeg',0)
		self.grey = np.zeros((im_width,im_height,1), np.uint8)
		self.cv_image_control = self.convert_img_to_cv(self.test_image)

    def test_pre_processing_grey_scale(self):
    	pre_processed_image = ipp.pre_processing(self.test_image)
    	cv_image_test = self.convert_img_to_cv(pre_processed_image)

		im_control_width, im_control_height, im_control_depth = self.cv_image_control.shape
    	im_test_width, im_test_height, im_test_depth = cv_image_test.shape

        self.assertEquals(im_control_depth, im_test_depth, "Image converts to greyscale. input_im_depth=%s, ctrl_im_depth=%s"%(im_test_depth, im_control_depth))

    def test_pre_processing_of_greyscale_input(self):
    	im_grey = ipp.pre_processing(self.test_image)
    	pre_processed_image = ipp.pre_processing(im_grey)
    	cv_image_test = self.convert_img_to_cv(pre_processed_image)

		im_control_width, im_control_height, im_control_depth = im_grey.shape
    	im_test_width, im_test_height, im_test_depth = cv_image_test.shape

        self.assertEquals(im_control_depth, im_test_depth, "Image converts from greyscale to greyscale. input_im_depth=%s, ctrl_im_depth=%s"%(im_test_depth, im_control_depth))

    def test_pre_processing_wrong_destination_im_size(self):
    	pre_processed_image = ipp.pre_processing(self.test_image)
    	cv_image_test = self.convert_img_to_cv(pre_processed_image)

    	im_test_width, im_test_height, im_test_depth = cv_image_test.shape
    	wrong_size_im = np.zeros((im_test_width*2,im_test_height*3,1), np.uint8)
    	wrong_size_im_width, wrong_size_im_height, wrong_size_im_depth = wrong_size_im.shape

    	self.assertEquals(im_test_width, wrong_size_im_width, "Image widths do not match. input_im_width=%s, wrong_size_im_width=%s"%(im_test_width, wrong_size_im_width))
    	self.assertEquals(im_test_height, wrong_size_im_height, "Image heights do not match. input_im_height=%s, wrong_size_im_height=%s"%(im_test_height, wrong_size_im_height))

    def test_pre_processing_correct_im_size(self):
   		pre_processed_image = ipp.pre_processing(self.test_image)
    	cv_image_test = self.convert_img_to_cv(pre_processed_image)

		iwrong_size_im_width, wrong_size_im_height, wrong_size_im_depth = self.cv_image_control.shape
    	im_test_width, im_test_height, im_test_depth = cv_image_test.shape

    	self.assertEquals(im_test_width, wrong_size_im_width, "Image widths match. input_im_width=%s, wrong_size_im_width=%s"%(im_test_width, wrong_size_im_width))
    	self.assertEquals(im_test_height, wrong_size_im_height, "Image heights match. input_im_height=%s, wrong_size_im_height=%s"%(im_test_height, wrong_size_im_height))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'unit_test_pre_processing', TestPreProcessing)