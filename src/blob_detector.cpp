/**
 * \file
 * \brief
 * \author
 * \version 0.1
 * \date
 *
 * \param[in]
 *
 * Subscribes to: <BR>
 *    °
 *
 * Publishes to: <BR>
 *    °
 *
 * Description
 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>

//ROS
#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs>

// Include here the ".h" files corresponding to the topic type you use.
// ...

using namespace cv;
using namespace std;

// You may have a number of globals here.
cv::Mat image_copy, mask_copy, morph_im;
bool is_image_available = false;
bool is_mask_available = false;

int morph_elem = 2;
int morph_size = 7;
int morph_operator = 0;
int const max_operator = 4;
int const max_elem = 2;
int const max_kernel_size = 21;

const char* window_name = "morphology";

// Callback functions...
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    is_image_available = true;
    try{
        // cv::imshow("original", cv_bridge::toCvShare(msg, "bgr8")->image);
        image_copy = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::waitKey(30);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

void maskCallback(const sensor_msgs::ImageConstPtr& msg)
{
    is_mask_available = true;
    try{
        // cv::imshow("original", cv_bridge::toCvShare(msg, "bgr8")->image);
        mask_copy = cv_bridge::toCvCopy(msg, "mono8")->image;
        cv::waitKey(30);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void Morphology_Operations( int, void* ) {}

int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "interactive_color_segment");

    // Define your node handles
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // Read the node parameters if any
    // ...

    // Declare your node's subscriptions and service clients
    image_transport::Subscriber im_subs = it.subscribe("image", 1, imageCallback);
    image_transport::Subscriber mask_subs = it.subscribe("mask", 1, maskCallback);

    // Declare you publishers and service servers
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float64MultiArray>("beacon_distance", 1);

    // node initilization
    // creating windows to display images
    namedWindow(window_name, WINDOW_NORMAL);
    namedWindow("keypoints", WINDOW_NORMAL);
    cv::startWindowThread();

    /// Create Trackbar to select Morphology operation
    createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat",
                   window_name, &morph_operator,
                   max_operator,
                   Morphology_Operations );
    /// Create Trackbar to select kernel type
    createTrackbar( "Element:\n 0: Rect - 1: Cross - 2: Ellipse",
                    window_name,
                    &morph_elem, max_elem,
                    Morphology_Operations );
    /// Create Trackbar to choose kernel size
    createTrackbar( "Kernel size:\n 2n +1", window_name,
                    &morph_size, max_kernel_size,
                    Morphology_Operations );

    // moving the windows to stack them horizontally
    int offset = 300;
    int initialX = 50;
    int initialY = 50;
    moveWindow(window_name, initialX + 2 * (offset + 5), initialY);
    moveWindow("keypoints", initialX + 4 * (offset + 5), initialY);

    // show all images initially
    Mat empty = Mat::zeros(480, 640, CV_8UC3);
    imshow("keypoints", empty);

    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    // Filter by Color
    params.filterByColor = true;
    params.blobColor = 255;

    // Change thresholds
    params.thresholdStep = 60;
    params.minThreshold = 20;
    params.maxThreshold = 200;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 300;
    params.maxArea = 160000;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.7;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.8;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.5;
    // Set up the detector with parameters.
#if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2
    SimpleBlobDetector detector(params);
#else
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
#endif
    std::vector<KeyPoint> keypoints;
    Mat im_with_keypoints, morph_with_keypoints;

    /// Default start
    Morphology_Operations( 0, 0 );

    // rate
    ros::Rate rate(30);
    while (ros::ok()){
        ros::spinOnce();

        // Your node's code goes here.
        if( !is_image_available || !is_mask_available){
            ROS_INFO("Waiting for image.") ;
            rate.sleep() ;
            continue ;
        }

        // Since MORPH_X : 2,3,4,5 and 6
        int operation = morph_operator + 2;

        Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
        /// Apply the specified morphology operation
        morphologyEx( mask_copy, morph_im, operation, element );

        // Detect blobs.
#if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2
	detector.detect(morph_im, keypoints);
	 for ( auto &keypoint : keypoints )
            keypoint.size *= 2;
#else
   	detector->detect(morph_im, keypoints);
#endif

        // Triangle similarity -> Focal_length = (distance2object * pixel_width) / actual_width
        // ROS_INFO("Distance: %0.2lf cm", 19700.0/keypoints[0].size );
        std_msgs::Float64MultiArray dist_msg;
        //dist_msg.data.push_back(19405.0/keypoints[0].size);
        dist_msg.data.push_back(0.00);
        dist_msg.data.push_back(20.00);
        dist_msg.data.push_back(40.00);
        dist_pub.publish( dist_msg );

        // Draw detected blobs as red circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        drawKeypoints( morph_im, keypoints, morph_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        drawKeypoints( image_copy, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

        // Show blobs
        imshow("keypoints", im_with_keypoints );
        imshow(window_name, morph_with_keypoints);


        rate.sleep();
    }
}
