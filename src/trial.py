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

def nothing(x):
    pass

class image_converter:

  def listener():
    #self.image_pub = rospy.Publisher("/camera/image_raw1",Image, queue_size=1)
    #self.image_pub3 = rospy.Publisher("/camera/image_raw3",Image, queue_size=1)

    rospy.Subscriber("/camera1/image_raw",Image,callback)#camera1/usb_cam/image_raw
    rospy.Subscriber("/camera3/image_raw",Image,callback1)

  def callback():
    print('left')
    try:
      cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    cv2.imshow('left',cv_image)
    cv2.waitKey(1)

  def callback1():
    print('right') 
    try:
      cv_image3 = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    cv2.imshow('right',cv_image3)  
    cv2.waitKey(1)

def main(args):
  ic = image_converter()
  rospy.init_node('image_processor', anonymous=True)
  listener()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    npz_calib_file_left = np.load('/home/sine/catkin_ws/src/vatsal/src/leftcam_fisheye_correction.npz')
    npz_calib_file_right = np.load('/home/sine/catkin_ws/src/vatsal/src/rightcam_fisheye_correction.npz')
    distCoeff_left=npz_calib_file_left['distCoeff']
    distCoeff_right=npz_calib_file_right['distCoeff']
    intrinsic_matrix_left = npz_calib_file_left['intrinsic_matrix']
    intrinsic_matrix_right = npz_calib_file_right['intrinsic_matrix']
    npz_calib_file_left.close()
    npz_calib_file_right.close()
    newMat_left, ROI_left = cv2.getOptimalNewCameraMatrix(intrinsic_matrix_left,distCoeff_left,(1280,720),alpha=5.5,centerPrincipalPoint=1)
    newMat_right, ROI_right = cv2.getOptimalNewCameraMatrix(intrinsic_matrix_right,distCoeff_right,(1280,720),alpha=5.7,centerPrincipalPoint=1)
    mapx_left,mapy_left=cv2.initUndistortRectifyMap(intrinsic_matrix_left,distCoeff_left,None,newMat_left,(1280,720),m1type = cv2.CV_32FC1)
    mapx_right,mapy_right=cv2.initUndistortRectifyMap(intrinsic_matrix_right,distCoeff_right,None,newMat_right,(1280,720),m1type = cv2.CV_32FC1)
main(sys.argv)