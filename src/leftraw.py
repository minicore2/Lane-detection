#!/usr/bin/env python
from __future__ import print_function
import roslib
#roslib.load_manifest('package.xml')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/camera/image_raw1",Image, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera1/image_raw",Image,self.callback)#camera1/usb_cam/image_raw

  def callback(self,data):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imshow('rawcamerainput',cv_image)
    cv2.waitKey(1)


def main(args):
  ic = image_converter()
  rospy.init_node('image_processor', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
	npz_calib_file = np.load('/home/sine/catkin_ws/src/vatsal/src/leftcam_fisheye_correction.npz')
	distCoeff=npz_calib_file['distCoeff']
	intrinsic_matrix = npz_calib_file['intrinsic_matrix']
	npz_calib_file.close()
	newMat, ROI = cv2.getOptimalNewCameraMatrix(intrinsic_matrix,distCoeff,(1280,720),alpha=5.5,centerPrincipalPoint=1)
	mapx,mapy=cv2.initUndistortRectifyMap(intrinsic_matrix,distCoeff,None,newMat,(1280,720),m1type = cv2.CV_32FC1)
main(sys.argv)


