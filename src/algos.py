#cv2.namedWindow('value')
#cv2.namedWindow('dilate')
#cv2.createTrackbar('bv', 'value', 120, 150, nothing)
#cv2.createTrackbar('cv', 'value', 40, 150, nothing)
#cv2.createTrackbar('ksize', 'value', 2, 100, nothing)
#cv2.createTrackbar('h', 'value', 10, 150, nothing)
#cv2.createTrackbar('templateWindowSize', 'value', 3, 50, nothing)
#cv2.createTrackbar('searchWindowSize', 'value', 10, 50, nothing)
#cv2.createTrackbar('val_th', 'value', 200, 255, nothing)
#cv2.createTrackbar('hlow_th', 'value', 40, 179, nothing)
#cv2.createTrackbar('hhigh_th', 'value', 60, 179, nothing)
#cv2.createTrackbar('ker_size_morph', 'value', 1, 50, nothing)
#cv2.createTrackbar('num_iter_morph', 'value', 1, 50, nothing)
#cv2.createTrackbar('iterations_dilate', 'dilate', 8, 50, nothing)
#cv2.createTrackbar('ker_size_dilate', 'dilate', 1, 50, nothing)

#ksize = 3
#kernel = np.ones((ksize,ksize),np.uint8)
#for i in range((ksize-1)/2,(ksize+1)/2):
	#for j in range((ksize-3)/2,(ksize+1)/2):
#kernel[1][1] = 0

 #hsv_image = cv2.cvtColor(im1, cv2.COLOR_BGR2HSV)
    #h, s, v = cv2.split(hsv_image)
    #cv2.imshow('hue',h)
    #cv2.imshow('saturation',s)
    #bv = cv2.getTrackbarPos('bv', 'value')
    #cv = cv2.getTrackbarPos('cv', 'value')
    #v_th = cv2.adaptiveThreshold(v,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
     #    cv2.THRESH_BINARY,(2*bv + 3),cv-100)

    #num_iter_morph = cv2.getTrackbarPos('num_iter_morph', 'value')
    #ker_size_morph = cv2.getTrackbarPos('ker_size_morph','value')
    #kernel_morph=cv2.getStructuringElement( cv2.MORPH_ELLIPSE, (2*ker_size_morph+1,2*ker_size_morph+1))
    #cv2.morphologyEx(v_th, cv2.MORPH_OPEN, kernel_morph, v_th,(-1,-1), num_iter_morph)

    #iterations_dilate = cv2.getTrackbarPos('iterations_dilate', 'dilate')
    #ker_size_dilate = cv2.getTrackbarPos('ker_size_dilate','dilate')
    #kernel_dilate=cv2.getStructuringElement( cv2.MORPH_ELLIPSE, (2*ker_size_dilate+1,2*ker_size_dilate+1)) 
    #cv2.dilate(v_th,kernel_dilate,v_th,(-1,-1),iterations_dilate)
    #cv2.imshow('dilate',v_th)
    
    #val_th = cv2.getTrackbarPos('val_th', 'value')
    #hlow_th = cv2.getTrackbarPos('hlow_th', 'value')
    #hhigh_th = cv2.getTrackbarPos('hhigh_th', 'value')
    #for i in range(0,439):
    #	for j in range(0,639):
    #		if v_th[i][j] == 1 and v[i][j] < val_th and h[i][j] < hhigh_th and h[i][j] > hlow_th:
    #			v_th[i][j]=1
    #		else:
    #			v_th[i][j]=0;

    #h = cv2.getTrackbarPos('h', 'value')
    #templateWindowSize = cv2.getTrackbarPos('templateWindowSize', 'value')*2 +1
    #searchWindowSize = cv2.getTrackbarPos('searchWindowSize', 'value')*2 + 1
    #dst = cv2.fastNlMeansDenoising(v_th, h, templateWindowSize, searchWindowSize)
    #ksize = cv2.getTrackbarPos('ksize','value')*2 + 1
    #dst =cv2.medianBlur(v_th, ksize)
    #cv2.imshow('value',v_th)

    sigmaSpace = cv2.getTrackbarPos('sigmaSpace','value')
    sigmaColor = cv2.getTrackbarPos('sigmaColor','value')
    #dst = cv2.medianBlur(v_th, ksize)
    #dst = cv2.boxFilter(v_th,-1, (ksize,ksize)
    dst = cv2.bilateralFilter(v_th, ksize, sigmaColor, sigmaSpace) 
    
import numpy as np
import cv2

img = cv2.imread('/home/sine/Pictures/vel.png',0)
dst = cv2.adaptiveBilateralFilter(img, 5, 5, maxSigmaColor = 5, anchor = (-1,-1))
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.imshow('image',dst)
cv2.waitKey(0)
cv2.destroyAllWindows()
