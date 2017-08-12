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

  def __init__(self):
    self.image_pub = rospy.Publisher("/camera/image_raw1",Image, queue_size=1)
    self.image_pub3 = rospy.Publisher("/camera/image_raw3",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera1/image_raw",Image,self.callback)#camera1/usb_cam/image_raw
    self.bridge3 = CvBridge()
    self.image_sub3 = rospy.Subscriber("/camera3/image_raw",Image,self.callback1)

  def callback(self,data):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #cv2.namedWindow('gaussorg')
    cv2.namedWindow('value-left')
    #cv2.namedWindow('dilate')
    cv2.createTrackbar('area-th', 'value-left', 25, 1000, nothing)
    #cv2.createTrackbar('b', 'gaussorg', 60, 150, nothing)
    #cv2.createTrackbar('c', 'gaussorg', 69, 150, nothing) 
    cv2.createTrackbar('bv', 'value-left', 60, 150, nothing)
    cv2.createTrackbar('cv', 'value-left', 60, 150, nothing)
    cv2.createTrackbar('ksize', 'value-left', 1, 20, nothing)
    #cv2.createTrackbar('iterations_dilate', 'dilate', 8, 50, nothing)
    #cv2.createTrackbar('ker_size_dilate', 'dilate', 2, 50, nothing)
    cv2.imshow('rawcamerainput',cv_image)

    #fisheye correction part
    global mapx_left,mapx_right,mapy_left,mapy_right;
    im1_left=cv2.remap(cv_image,mapx_left,mapy_left,cv2.INTER_LINEAR)
    im1_right=cv2.remap(cv_image3,mapx_right,mapy_right,cv2.INTER_LINEAR)
    #fisheye ends
    im1_left=cv2.resize(im1_left,(640,480))
    im1_left=im1_left[40:480,0:640]
    im1_right=cv2.resize(im1_right,(640,480))
    im1_right=im1_right[40:480,0:640]
    #cv2.namedWindow('fisheyeleft')
    #cv2.imshow('fisheye_correction',im1)

    b_left, g_left, r_left = cv2.split(im1_left)
    b_right, g_right, r_right = cv2.split(im1_right)
    #cv2.imshow('blue',b)
    #cv2.imshow('green',g)
    #cv2.imshow('red',r)
    #hsv_image = cv2.cvtColor(im1, cv2.COLOR_BGR2HSV)
    #h, s, v = cv2.split(hsv_image)
    bv = cv2.getTrackbarPos('bv', 'value-left')
    cv = cv2.getTrackbarPos('cv', 'value-left')
    bv3 = cv2.getTrackbarPos('bv3', 'value-right')
    cv3 = cv2.getTrackbarPos('cv3', 'value-right')
    v_lth = cv2.adaptiveThreshold(b_left,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
         cv2.THRESH_BINARY,(2*bv + 3),cv-100)
    v_rth = cv2.adaptiveThreshold(b_right,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
         cv2.THRESH_BINARY,(2*bv + 3),cv-100)
    ksize = cv2.getTrackbarPos('ksize', 'value-left')*2 + 1
    ksize3 = cv2.getTrackbarPos('ksize3', 'value-right')*2 + 1
    dst_left = cv2.medianBlur(v_lth, ksize)
    dst_right = cv2.medianBlur(v_rth, ksize3)
    cv2.imshow('value-left',dst_left)
    cv2.imshow('value-right',dst_right)
    #iterations_dilate = cv2.getTrackbarPos('iterations_dilate', 'dilate')
    #ker_size_dilate = cv2.getTrackbarPos('ker_size_dilate','dilate')
    #kernel_dilate=cv2.getStructuringElement( cv2.MORPH_ELLIPSE, (2*ker_size_dilate+1,2*ker_size_dilate+1)) 
    #cv2.dilate(dst,kernel_dilate,dst,(-1,-1),iterations_dilate)
    #cv2.imshow('dilate',dst)
    


    #cv2.imshow('areathl',im1)
    #im1 = cv2.blur(im1,(5,5))
    blank = np.zeros((im1_left.shape[0],im1_left.shape[1],3), np.uint8)
    blank3 = np.zeros((im1_right.shape[0],im1_right.shape[1],3), np.uint8)
    #blank2 = np.zeros((im1.shape[0],im1.shape[1],3), np.uint8)
    rows,cols = blank.shape[:2]
    #img = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)

    #b = cv2.getTrackbarPos('b', 'gaussorg')
    #c = cv2.getTrackbarPos('c', 'gaussorg')
    #thgauss = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
         #cv2.THRESH_BINARY,(2*b + 3),c-100)
    #cv2.imshow('gaussorg',thgauss)
    #cv2.rectangle(im1,(240,330),(400,480),0,-1)#CV_FILLED=-1
	
    thgauss,contours,hierarchy = cv2.findContours(dst_left,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)#thgauss->v_th->dst
    thgauss3,contours,hierarchy = cv2.findContours(dst_right,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    area = cv2.getTrackbarPos('area-th', 'value-left')
    area3 = cv2.getTrackbarPos('area-th3', 'value-right')

    for n, contours in enumerate(contours):
        if cv2.contourArea(contours) > area :
            #x,y,w,h = cv2.boundingRect(contours)
            #if True :
                #cv2.rectangle(im1,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.drawContours(im1_left,[contours],-1,(0,255,0),2)            
            cv2.drawContours(blank,[contours],-1,(255,255,255),-1)
            cv2.drawContours(im1_right,[contours],-1,(0,255,0),2)            
            cv2.drawContours(blank3,[contours],-1,(255,255,255),-1)

            '''extLeft = tuple(contours[contours[:, :, 0].argmin()][0])
                extRight = tuple(contours[contours[:, :, 0].argmax()][0])
                [vx,vy,x,y] = cv2.fitLine(contours, cv2.DIST_L2,0,0.01,0.01)
                lefty = int(((extLeft[0]-x)*vy/vx) + y)
                righty = int(((extRight[0]-x)*vy/vx)+y)
                cv2.line(blank,(extRight[0]-1,righty),(extLeft[0],lefty),(255,255,255),10)'''
                
                
    final = cv2.cvtColor(blank, cv2.COLOR_BGR2GRAY)
    final3 = cv2.cvtColor(blank3, cv2.COLOR_BGR2GRAY)

    #cv2.namedWindow('im1')
    cv2.imshow('fisheyeleft',im1_left)
    #cv2.namedWindow('Contour-left')
    cv2.imshow('Contour-left',final)
    cv2.waitKey(1)
  

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(final, "mono8"))
      self.image_pub3.publish(self.bridge3.cv2_to_imgmsg(final3, "mono8"))
    except CvBridgeError as e:
      print(e)

  def callback1(self,data):
    try:
      cv_image3 = self.bridge3.imgmsg_to_cv2(data,"bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.namedWindow('value-right')

def main(args):
  ic = image_converter()
  rospy.init_node('image_processor', anonymous=True)
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


