#include "image_properties.h"
#include "nav_msgs/OccupancyGrid.h"
#include "math.h"

using namespace cv;
using namespace std

image_transport::Publisher pub;

Mat src;
int area_th = 1020;
int sigma = 60;
int c = 69;

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "adaptive";
static const char TRKBAR[] = "Trackbar_Window";

void on_trackbar(int , void*){
	 src=resize(src,(640,480));
}

void imageCallback(const sensor_msgs::ImageConstPtr& original_image){
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    createTrackbar("area_th", TRKBAR, &k, 3000, on_trackbar);
    createTrackbar("sigma", TRKBAR, &th, 100, on_trackbar);
    createTrackbar("c", TRKBAR, &sigma1, 100, on_trackbar);

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		//Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image , enc::BGR8);
	}
	catch(cv_bridge::Exception& e)
	{
		//if there is an error during conversion, display it
		ROS_ERROR("videofeed::main.cpp::cv_bridge exception: %s", e.what());
		 return;
	};
	src = cv_ptr->image;
	on_trackbar(0, 0);
    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window 
	//created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);
    //imshow("source_window",src);
    //imshow(WINDOW,filtered);
    //imshow("filter1",filter1);
    //imshow("filter2",filter2);
    //imshow("src_gray",src_gray);
    /**
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise<>() call, as was done
    * in the constructor in main().
    */
    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
        //pub.publish(cv_ptr->toImageMsg());
}

/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);
    //OpenCV HighGUI call to create a display window on start-up.
    namedWindow("source_window", CV_WINDOW_AUTOSIZE );
    //namedWindow("gray scale", CV_WINDOW_AUTOSIZE );
    namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    namedWindow(TRKBAR, CV_WINDOW_AUTOSIZE);
    /**
    * Subscribe to the "camera/image_raw" base topic. The actual ROS topic subscribed to depends on which transport is used.
    * In the default case, "raw" transport, the topic is in fact "camera/image_raw" with type sensor_msgs/Image. ROS will call
    * the "imageCallback" function whenever a new image arrives. The 2nd argument is the queue size.
    * subscribe() returns an image_transport::Subscriber object, that you must hold on to until you want to unsubscribe.
    * When the Subscriber object is destructed, it will automatically unsubscribe from the "camera/image_raw" base topic.
    */
    //image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw",1,imageCallback);
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw",1,imageCallback);

    //OpenCV HighGUI call to destroy a display window on shut-down.
    //cv::destroyWindow(WINDOW);
    /**
    * The advertise() function is how you tell ROS that you want to
    * publish on a given topic name. This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing. After this advertise() call is made, the master
    * node will notify anyone who is trying to subscribe to this topic name,
    * and they will in turn negotiate a peer-to-peer connection with this
    * node.  advertise() returns a Publisher object which allows you to
    * publish messages on that topic through a call to publish().  Once
    * all copies of the returned Publisher object are destroyed, the topic
    * will be automatically unadvertised.
    *
    * The second parameter to advertise() is the size of the message queue
    * used for publishing messages.  If messages are published more quickly
    * than we can send them, the number here specifies how many messages to
    * buffer up before throwing some away.
    */
    pub = it.advertise("/camera/image_raw1",1);
    ros::Rate loop_rate(15);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", filter2).toImageMsg();
        pub.publish(msg);
    }

    /**
    * In this application all user callbacks will be called from within the ros::spin() call.
    * ros::spin() will not return until the node has been shutdown, either through a call
    * to ros::shutdown() or a Ctrl-C.
    */
    ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("videofeed::main.cpp::No error.");
}