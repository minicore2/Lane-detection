#include "image_properties.h"
//ros libraries
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
//c++ libraries
#include <iostream>
#include <vector>
#include <math.h>
#include <string>
#include <fstream>


using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

ros::Subscriber sub_Lanedata_right;
image_transport::Publisher pub_right;

int bv = 60;
int c = 60;
int ksize = 1;
int area = 25;
int hue_low = 79;
int hue_high = 179;
int sat_low = 23;
int sat_high = 255;
int val_low = 0;
int val_high = 36;

std::vector<std::vector<Point> > contours;
std::vector<Vec4i> hierarchy;
std::vector<std::vector<Point> > contours_blue;
std::vector<Vec4i> hierarchy_blue;

Mat src,temp1,temp2,temp3,temp4,blank,dst,blue,map1_right,map2_right;
vector<Mat> BGR;

void removeblue(Mat bluebarrel)
{	
	Mat bw;
	namedWindow("test_right",CV_WINDOW_AUTOSIZE);
	createTrackbar("hue_low","test_right", &hue_low, 179);
	createTrackbar("hue_high","test_right", &hue_high, 179);
	createTrackbar("sat_low","test_right", &sat_low, 255);
	createTrackbar("sat_high","test_right", &sat_high, 255);
	createTrackbar("val_low","test_right", &val_low, 255);
	createTrackbar("val_high","test_right", &val_high, 255);
	hue_low = getTrackbarPos("hue_low","test_right");
	hue_high = getTrackbarPos("hue_high","test_right");
	sat_low = getTrackbarPos("sat_low","test_right");
	sat_high = getTrackbarPos("sat_high","test_right");
	val_low = getTrackbarPos("val_low","test_right");
	val_high = getTrackbarPos("val_high","test_right");
	inRange(bluebarrel,Scalar(hue_low,sat_low,val_low),Scalar(hue_high,sat_high,val_high),bw);//detects blue
	imshow("test_right",bw);
	findContours( bw, contours_blue, hierarchy_blue, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
}

void rightimage(const sensor_msgs::ImageConstPtr& original_image)
{
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("videofeed::igvc_IPM.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    src = cv_ptr->image;
    namedWindow("Final_right", CV_WINDOW_AUTOSIZE);
    //namedWindow("test_right",CV_WINDOW_AUTOSIZE);
    createTrackbar("bv", "Final_right", &bv, 150);
	createTrackbar("c", "Final_right", &c, 150);
	createTrackbar("kernelsize","Final_right", &ksize, 20);
	createTrackbar("area","Final_right", &area, 1000);
	// createTrackbar("us","test_right", &us, 200);
	
	remap(src, temp1, map1_right, map2_right, CV_INTER_LINEAR);
	resize(temp1, temp1, Size (640,480));
	temp2 = temp1(Rect(0,40,640,440));
	imshow("original_right",temp2);
	removeblue(temp2);
	split(temp2,BGR);
	blue = BGR[0];
	//imshow("blue",blue);
	bv = getTrackbarPos("bv", "Final_right");
	c = getTrackbarPos("c","Final_right");
	adaptiveThreshold(blue, temp3, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, (2*bv + 3), c-100);
	//drawContours( temp3, contours_blue, -1, Scalar(0), -1);
	rectangle(temp3, Point(160,370),Point(280,440), 0,-1);//280
	//imshow("gauss",temp3);
	ksize = getTrackbarPos("kernelsize","Final_right");
	medianBlur(temp3, temp4, ksize*2 + 1);

	findContours(temp4, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
	blank = Mat::zeros(temp4.rows, temp4.cols, CV_8UC1);
	area = getTrackbarPos("area", "Final_right");
	for (int i = 0; i < contours.size(); ++i)
    {
        if (contourArea(contours[i]) > area)
        {
        	drawContours(blank, contours, i, Scalar(255),-1);
        }
    }
	imshow("Final_right",blank);
	cvtColor(blank, dst, CV_GRAY2BGR);
	cv_ptr->image = dst;
    pub_right.publish(cv_ptr->toImageMsg());
	waitKey(1);
}


int main(int argc, char **argv)
{	
    ros::init(argc, argv, "Lane_r");
    ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);

	image_transport::Subscriber sub_left = it.subscribe("/camera3/image_raw", 1, rightimage);
	//image_transport::Subscriber sub_right = it.subscribe("/camera3/image_raw", 1, rightimage);

	//pub_Lanedata = nh.advertise<nav_msgs::OccupancyGrid>("/Lane_Occupancy_Grid", 1);
	pub_right = it.advertise("/camera/image_raw3", 1);
	ros::Rate loop_rate(20);

	Rect* LeftROI;

	std::ifstream matright("/home/sine/catkin_ws/src/vatsal/src/intrinsic_matrix_right.txt",std::ios::in);
	std::ifstream distright("/home/sine/catkin_ws/src/vatsal/src/distCoeff_right.txt",std::ios::in);
	
	std::vector<double> a;
	std::vector<double> distCoeff_right;
	double num1 = 0.0;
	while (matright >> num1) {
        a.push_back(num1);
    }
    double num2 = 0.0;
    while (distright >> num2) {
        distCoeff_right.push_back(num2);
    }
    matright.close();
    distright.close();
    Mat intrinsic_matrix_right = (Mat_<double>(3,3) << a[0], a[1], a[2], a[3],a[4], a[5], a[6], a[7], a[8]);
    Mat newmat_right = getOptimalNewCameraMatrix(intrinsic_matrix_right, distCoeff_right,Size (1280,720), 5.7,Size (1280,720), LeftROI, true);
    initUndistortRectifyMap(intrinsic_matrix_right, distCoeff_right, Mat::eye(3, 3, CV_8U), newmat_right, Size (1280,720), CV_32FC1, map1_right, map2_right);
    a.clear();
    distCoeff_right.clear();
    newmat_right.release();
    intrinsic_matrix_right.release();
	while(ros::ok())
	{	
		//begin = ros::Time::now().toSec();
		ros::spinOnce();
		//end = ros::Time::now().toSec();
		//cout<< end - begin <<endl;
		//final_grid.release();
		loop_rate.sleep();
		
	}

	ROS_INFO("videofeed::occupancygrid.cpp::No error.");

}