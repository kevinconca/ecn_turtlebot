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
turtle_ekf::Pose2DWithCovariance odom_2D;
bool is_odom2D_available=false;

// Callback functions...
void odom_Callback( turtle_ekf::Pose2DWithCovariance odom_msg) {
    is_odom2D_available= true;
    odom_2D = odom_msg;
}

void ref_Callback( turtle_ekf::Pose2DWithCovariance odom_msg) {
    is_odom2D_available= true;
    odom_2D = odom_msg;
}


int main (int argc, char** argv){

    //ROS Initialization
    ros::init(argc, argv, "controller");

    // Define your node handles
    ros::NodeHandle nh_loc("~"), nh_glob;

    // Read the node parameters if any

    // Declare your node's subscriptions and service clients
    ros::Subscriber RoborPose_sub = nh_glob.subscribe<turtle_ekf::Pose2DWithCovariance>("/robot_pose_ekf",1, odom_Callback) ;
    ros::Subscriber ref_sub = nh_glob.subscribe<turtle_ekf::Pose2DWithCovariance>("/reference_trajectory",1, ref_Callback) ;

    // Declare you publishers and service servers

    // node initilization

    ros::Rate rate(10);

    while (ros::ok()){

        ros::spinOnce();

        if( !is_odom2D_available ){
                         ROS_INFO("Waiting for /odom.") ;
                         rate.sleep() ;
                         continue ;
                     }
        rate.sleep();
    }


}
