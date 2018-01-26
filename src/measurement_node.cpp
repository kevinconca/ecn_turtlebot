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
#include <cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>

//ROS
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs>


using namespace cv;
// Include here the ".h" files corresponding to the topic type you use.
// ...

// You may have a number of globals here.
cv::Mat image_copy, filtered_im;
bool is_image_available = false;

const char* window_name= "Edge Map";


int morph_elem = 0;
int morph_size = 0;
int morph_operator = 0;
int const max_operator = 4;
int const max_elem = 2;
int const max_kernel_size = 21;


// Callback functions...
//-------------------------------------------------------------------------------------------------
void Morphology_Operations( int, void* )
{
  // Since MORPH_X : 2,3,4,5 and 6
  int operation = morph_operator + 2;

  Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );

  /// Apply the specified morphology operation
  morphologyEx( filtered_im, filtered_im, operation, element );
  imshow( window_name, filtered_im );

  }
//-------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      is_image_available = true;
      try{
       //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
       image_copy= cv_bridge::toCvCopy(msg, "bgr8")->image;
       cv::waitKey(30);
     }catch (cv_bridge::Exception& e){
       ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
   }
//----------------------------------------------------------------------------------------------------


int main (int argc, char** argv)
{

   //ROS Initialization
    ros::init(argc, argv, "measurement_node");

    // Define your node handles
   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
    // Read the node parameters if any

    // Declare your node's subscriptions and service clients
    image_transport::Subscriber sub = it.subscribe("/occam/image0", 1, imageCallback);

    // Declare you publishers and service servers
    // node initilization

     cv::namedWindow("copy");
     cv::namedWindow("keypoints");
     namedWindow( window_name, CV_WINDOW_AUTOSIZE );
     cv::startWindowThread();

     cv::Mat HSV, H, S, V;

    // rate 
    ros::Rate rate(10);  
 //---------------------------------------------------------------
	while (ros::ok()){
            ros::spinOnce();
        //test condition
            if( !is_image_available ){
                 ROS_INFO("Waiting for image.") ;
                 rate.sleep() ;
                 continue ;
             }

        // Your node's code goes here.

           //cv::split(image_copy,bgr);//split source

            cv::cvtColor(image_copy, HSV, CV_BGR2HSV);
            std::vector<cv::Mat> hsv_planes;
            cv::split(HSV, hsv_planes);
            H = hsv_planes[0];
            S = hsv_planes[1];
            V = hsv_planes[2];
            //cv::imshow("blue", H);

            //Mat filtered_im;
            //cv::inRange(HSV, Scalar(105, 50, 50), Scalar(115, 255, 255), filtered_im);  //blue

            cv::inRange(HSV, Scalar(13, 85, 109), Scalar(28, 132, 255), filtered_im);  //red



            /// Create Trackbar to select Morphology operation
             createTrackbar("Operator:\n 0:Op  1: Cl \n 2: Gr - 3:T_H \n 4:B_H", window_name, &morph_operator, max_operator, Morphology_Operations );

             /// Create Trackbar to select kernel type
             createTrackbar( "Element:\n 0: Rect - 1: Cross - 2: Ellipse", window_name, &morph_elem, max_elem, Morphology_Operations );

             /// Create Trackbar to choose kernel size
             createTrackbar( "Kernel size:\n 2n +1", window_name, &morph_size, max_kernel_size, Morphology_Operations );


            // Setup SimpleBlobDetector parameters.
            SimpleBlobDetector::Params params;
            params.filterByColor = true;
            params.blobColor = 255;

//            // Change thresholds
//            params.minThreshold = 10;
//            params.maxThreshold = 200;

            // Filter by Area.
            params.filterByArea = true;
            params.minArea = 1500;
            params.maxArea = 160000;

//            // Filter by Circularity
//            params.filterByCircularity = true;
//            params.minCircularity = 0.1;

//            // Filter by Convexity
//            params.filterByConvexity = true;
//            params.minConvexity = 0.87;

            // Filter by Inertia
           // params.filterByInertia = true;
           // params.minInertiaRatio = 0.1;

            #if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2

              // Set up detector with params
              SimpleBlobDetector detector(params);

            #else

              // Set up detector with params
              Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
            #endif

              //cv::SimpleBlobDetector detector;

              /// Default start
              Morphology_Operations( 0, 0 );


            // Detect blobs.
            std::vector<KeyPoint> keypoints;
            detector->detect( filtered_im, keypoints);

            // Draw detected blobs as red circles.
            // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
            cv::Mat im_with_keypoints;
            cv::drawKeypoints( filtered_im, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

            // Show blobs
            imshow("keypoints", im_with_keypoints );

             imshow("copy", H );

//              /// Create a matrix of the same type and size as src (for dst)
//              dst.create( image_copy.size(), image_copy.type() );

//              /// Create a window
//              cv::namedWindow( window_name, CV_WINDOW_AUTOSIZE );

//              /// Create a Trackbar for user to enter threshold
//              cv::createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

//              /// Show the image
//              CannyThreshold(0, 0);


		rate.sleep();
    }
}
