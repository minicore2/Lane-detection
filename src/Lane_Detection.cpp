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

ros::Subscriber sub_Lanedata_left;
ros::Subscriber sub_Lanedata_right;
//Use method of ImageTransport to create image publisher
image_transport::Publisher pub_left;

int bv = 60;
int c = 60;
int ksize = 1;
int area = 25;

std::vector<std::vector<Point> > contours;
std::vector<Vec4i> hierarchy;

Mat src,temp1,temp2,temp3,temp4,blank,dst,blue,map1_left,map2_left;
vector<Mat> BGR;

void leftimage(const sensor_msgs::ImageConstPtr& original_image)
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
    namedWindow("Final_left", CV_WINDOW_AUTOSIZE);
    //namedWindow("test",CV_WINDOW_AUTOSIZE);
    createTrackbar("bv", "Final_left", &bv, 150);
	createTrackbar("c", "Final_left", &c, 150);
	createTrackbar("kernelsize","Final_left", &ksize, 20);
	createTrackbar("area","Final_left", &area, 1000);
	// createTrackbar("us","test", &us, 200);
	
	remap(src, temp1, map1_left, map2_left, CV_INTER_LINEAR);
	resize(temp1, temp1, Size (640,480));
	imshow("original_left",temp1);
	temp2 = temp1(Rect(0,40,640,440));
	split(temp2,BGR);
	blue = BGR[0];

	bv = getTrackbarPos("bv", "Final_left");
	c = getTrackbarPos("c","Final_left");
	adaptiveThreshold(blue, temp3, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, (2*bv + 3), c-100);
	ksize = getTrackbarPos("kernelsize","Final_left");
	medianBlur(temp3, temp4, ksize*2 + 1);

	findContours(temp4, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
	blank = Mat::zeros(temp4.rows, temp4.cols, CV_8UC1);
	area = getTrackbarPos("area", "Final_left");
	for (int i = 0; i < contours.size(); ++i)
    {
        if (contourArea(contours[i]) > area)
        {
        	drawContours(blank, contours, i, Scalar(255),-1);
        }
    }
	imshow("Final_left",blank);
	cout<<blank.size()<<endl;
	cv_ptr->image = blank;
    pub_left.publish(cv_ptr->toImageMsg());
    waitKey(1);
}


int main(int argc, char **argv)
{	
    ros::init(argc, argv, "Lane_l");
    ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);

	image_transport::Subscriber sub_left = it.subscribe("/camera1/image_raw", 1, leftimage);
	//image_transport::Subscriber sub_right = it.subscribe("/camera3/image_raw", 1, rightimage);

	//pub_Lanedata = nh.advertise<nav_msgs::OccupancyGrid>("/Lane_Occupancy_Grid", 1);
	pub_left = it.advertise("/camera/image_raw1", 1);
	ros::Rate loop_rate(10);

	Rect* LeftROI;

	std::ifstream matleft("/home/sine/catkin_ws/src/vatsal/src/intrinsic_matrix_left.txt",std::ios::in);
	std::ifstream distleft("/home/sine/catkin_ws/src/vatsal/src/distCoeff_left.txt",std::ios::in);
	// std::ifstream matright("/home/vatsal/catkin_ws/src/vatsal/src/intrinsic_matrix_right.txt",std::ios::in);
	// std::ifstream distright("/home/vatsal/catkin_ws/src/vatsal/src/distCoeff_right",std::ios::in);
	
	std::vector<double> a;
	std::vector<double> distCoeff_left;
	double num1 = 0.0;
	while (matleft >> num1) {
        a.push_back(num1);
    }
    double num2 = 0.0;
    while (distleft >> num2) {
        distCoeff_left.push_back(num2);
    }
    matleft.close();
    distleft.close();
    Mat intrinsic_matrix_left = (Mat_<double>(3,3) << a[0], a[1], a[2], a[3],a[4], a[5], a[6], a[7], a[8]);
    Mat newmat_left = getOptimalNewCameraMatrix(intrinsic_matrix_left, distCoeff_left,Size (1280,720), 5.5,Size (1280,720), LeftROI, true);
    initUndistortRectifyMap(intrinsic_matrix_left, distCoeff_left, Mat::eye(3, 3, CV_8U), newmat_left, Size (1280,720), CV_32FC1, map1_left, map2_left);
    a.clear();
    distCoeff_left.clear();
    newmat_left.release();
    intrinsic_matrix_left.release();
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