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
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
using Eigen::MatrixXd;

//ROS
#include "ros/ros.h"

// Include here the ".h" files corresponding to the topic type you use.
#include <nav_msgs/Odometry.h>
#include <turtle_ekf/Pose2DWithCovariance.h>
#include "tf/transform_datatypes.h"


// global variables...
nav_msgs::Odometry odom_3D;
bool is_odom3D_available=false;

// Callback functions...
void odom_Callback( nav_msgs::Odometry odom_msg) {
    is_odom3D_available= true;
    odom_3D = odom_msg;
}


int main (int argc, char** argv){

    //ROS Initialization
    ros::init(argc, argv, "odom_interface");

    // Define your node handles
    ros::NodeHandle nh_loc("~"), nh_glob;

    // Read the node parameters if any

    // Declare your node's subscriptions and service clients
    ros::Subscriber odom_sub = nh_glob.subscribe<nav_msgs::Odometry>("/odom",1, odom_Callback) ;

    // Declare you publishers and service servers
    ros::Publisher  odom2D_pub = nh_glob.advertise<turtle_ekf::Pose2DWithCovariance>("/odom2D",1) ;

    // node initilization
    turtle_ekf::Pose2DWithCovariance odom_2D;
    ros::Rate rate(10);

    while (ros::ok()){

        ros::spinOnce();

        if( !is_odom3D_available ){
                         ROS_INFO("Waiting for /odom.") ;
                         rate.sleep() ;
                         continue ;
                     }

     // extractt 2D pose
        tf::Quaternion q(odom_3D.pose.pose.orientation.x, odom_3D.pose.pose.orientation.y, odom_3D.pose.pose.orientation.z, odom_3D.pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        odom_2D.pose.x= 1+ odom_3D.pose.pose.position.x;
        odom_2D.pose.y= 1+ odom_3D.pose.pose.position.y;
        odom_2D.pose.theta=yaw + 0; //M_PI/2;;

     // extract 3*3 covariance matrix
        odom_2D.Covariance[0]= .05; //odom_3D.pose.covariance[0];
        odom_2D.Covariance[1]= 0;
        odom_2D.Covariance[2]= 0;
        odom_2D.Covariance[3]= 0;
        odom_2D.Covariance[4]= .05; //odom_3D.pose.covariance[7];
        odom_2D.Covariance[5]= 0;
        odom_2D.Covariance[6]= 0;
        odom_2D.Covariance[7]= 0;
        odom_2D.Covariance[8]= .05; //odom_3D.pose.covariance[35];
    // header
        odom_2D.header.seq=odom_3D.header.seq;
        odom_2D.header.stamp=odom_3D.header.stamp;
        odom_2D.header.frame_id=odom_3D.header.frame_id;
    // publish 2D pose ........................................................................................................
        odom2D_pub.publish(odom_2D);



        rate.sleep();
    }


}
