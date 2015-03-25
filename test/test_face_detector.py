#!/usr/bin/env python

PKG='offloadable_face_recognition'
import sys
import unittest
import cv2
from face_detector import Face_Detector as fd
from offloadable_fr_node import Offloadable_FR_Node
from offloadable_face_recognition.msg import FaceBox
from sensor_msgs.msg import Image

## A sample python unit test
class TestFaceDetector(unittest.TestCase):
    def set_up(self):
        super(TestFaceDetector, self).__init__(*args)
        # Harr cascade classifiers
        
        self.c1 = cv2.CascadeClassifier("haarcascade_frontalface_alt.xml")
        self.c2 = cv2.CascadeClassifier("haarcascade_frontalface_alt")
        self.c3 = cv2.CascadeClassifier("haarcascade_profileface.xml")

        self.test_image_face = self.convert_cv_to_img(cv2.imread('test_face.jpg',0))
        self.test_image_no_face = self.convert_cv_to_img(cv2.imread('test_no_face.jpg',0))
        self.no_image = np.zeros((0,0), np.uint8)
        self.test_images_good = self.import_test_images
        self.test_images_bad = self.generate_bad_images

        def import_test_images(self):
            test_images = []
            prefix = "i"
            filetype = ".jpg"
            for suffix in range(0,12):
                im_name = prefix+str(suffix)+filetype
                im = self.convert_cv_to_img(cv2.imread(im_name,0))
                test_images.append
            return test_images

        def generate_bad_images(self):
            good images = self.import_test_images()
            bad_images = good_images[0:6]
            bad_images.append(self.no_image)
            bad_images = good_images[7:11]
            bad_images.append(self.no_image)
            return bad_images

        def test_face_detector_face(self):
            fd.set_classifiers(c1, c2, c3)
            face_box = fd.detect_face(self.test_image_face)
            if face_box is not None:
            x1 = face_box.x
            y1 = face_box.y
            x2 = face_box.width + x1
            y2 = face_box.height + y1
            self.assertTrue(face_box is not None, "Image contains a face between (%d,%d) and (%d,%d)"%(x1,y1,x2,y2))

        def test_face_detector_no_face(self):
            fd.set_classifiers(c1, c2, c3)
            face_box = fd.detect_face(self.test_image_face)
            x1 = face_box.x
            y1 = face_box.y
            x2 = face_box.width + x1
            y2 = face_box.height + y1
            self.assertTrue(face_box is None, "Image does not contain a face!")

        def test_multiple_faces(self, faces):
            fd.set_classifiers(c1, c2, c3)
            for image in faces:
                index = 0
                face_box = fd.detect_face(image)
                if face_box is not None:
                    x1 = face_box.x
                    y1 = face_box.y
                    x2 = face_box.width + x1
                    y2 = face_box.height + y1

                    self.assertTrue(face_box is not None, "Image %d contains a face between (%d,%d) and (%d,%d)"%(index,x1,y1,x2,y2))
                else:
                    self.assertTrue(face_box is None, "Image %d contains no face!"%s(index))
                index+=1

        def test_face_detector_multiple_good_faces(self):
            self.test_multiple_faces(self.test_images_good)

        def test_face_detector_multiple_bad_faces(self):
            self.test_multiple_faces(self.test_images_bad)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'unit_test_face_detector', TestFaceDetector)