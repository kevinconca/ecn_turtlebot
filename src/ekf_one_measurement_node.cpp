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
#include "tf/transform_datatypes.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/Float64MultiArray.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

//#include <geometry_msgs/PoseWithCovariance.h>
//#include <std_msgs/Float32.h>

// global variables...
nav_msgs::Odometry odom;
std_msgs::Float64MultiArray dist;
bool is_odom_available= false, is_dist_available=false;


// Callback functions...
void odom_Callback( nav_msgs::Odometry odom_msg) {
    is_odom_available= true;
    odom = odom_msg;
}

void msurmnt_Callback ( std_msgs::Float64MultiArray  dist_msg) {
    is_dist_available= true;
    dist = dist_msg;
}


int main (int argc, char** argv){

    //ROS Initialization
    ros::init(argc, argv, "ekf_node");

    // Define your node handles
    ros::NodeHandle nh_loc("~"), nh_glob;

    // Read the node parameters if any

    // Declare your node's subscriptions and service clients
    ros::Subscriber odom_sub = nh_glob.subscribe<nav_msgs::Odometry>("/odom",1, odom_Callback) ;
    ros::Subscriber msurmnt_sub = nh_glob.subscribe<std_msgs::Float64MultiArray>("/beacon_distance",1, msurmnt_Callback) ;

    // Declare you publishers and service servers
    ros::Publisher  pose_pub = nh_glob.advertise<geometry_msgs::Pose2D>("/pose",1) ;

    // node initilization
    MatrixXd Q_alpha =  MatrixXd::Zero(3,3);
    MatrixXd Q_gamma =  MatrixXd::Zero(1,1);
    MatrixXd P =  MatrixXd::Zero(3,3);
    MatrixXd C =  MatrixXd::Zero(1,3);
    MatrixXd D =  MatrixXd::Zero(1,1);
    MatrixXd K =  MatrixXd::Zero(3,1);
    MatrixXd I =  MatrixXd::Zero(3,3);
    I(0,0)= 1;   I(1,1)= 1;     I(2,2)= 1;

    VectorXd X(3);
    VectorXd Y(1);
    VectorXd Y_hat(1);

    geometry_msgs::Point B1, B2, B3;
    B1.x=0;  B1.y=10; B2.x=10;  B2.y=0; B3.x=20; B3.y=20;


    ros::Rate rate(10);

    while (ros::ok()){

        ros::spinOnce();
                if( ! is_dist_available || !is_odom_available ){
                    ROS_INFO("Waiting for odom/dist.") ;
                    rate.sleep() ;
                    continue ;
                }

        // odometry part:
        // Prediction step: predicted_state Xk+1/k "extract pose from odom"
        X(0)=odom.pose.pose.position.x;
        X(1)=odom.pose.pose.position.y;
        tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        X(2)=yaw;

        // state_trans_uncertainty_noise: Q_alpha "extract covariance matrix from odom"
        Q_alpha(0,0)= 1; //odom.pose.covariance[0];
        Q_alpha(1,1)= .1; //odom.pose.covariance[7];
        Q_alpha(2,2)= .1; //odom.pose.covariance[35];

        // jacobian_matrix: A = I

        // pred_covariance_est: Pk+1/k = A. PK/k. A^T + Q_alpha  "P= A*P*A.transpose() + Q_alpha;" but A=I
        P=P+Q_alpha;

        // measurement part:

        // Actual measurement  Y
        Y(0)= dist.data[0];
        //Y(1)= dist.data[1];
        //Y(2)= dist.data[2];

        // Expected measurement Y_hat
        Y_hat(0)= sqrt( (B1.x-X(0))*(B1.x-X(0)) + (B1.y-X(1))*(B1.y-X(1)) );
        //Y_hat(1)= sqrt( (B2.x-X(0))*(B2.x-X(0)) + (B2.y-X(1))*(B2.y-X(1)) );
        //Y_hat(2)= sqrt( (B3.x-X(0))*(B3.x-X(0)) + (B3.y-X(1))*(B3.y-X(1)) );

        // Measurement jacobian C_matrix
        C(0,0)= 2*(X(0) - B1.x) / Y_hat(0);
        C(0,1)= 2*(X(1) - B1.y) / Y_hat(0);
//        C(1,0)= 2*(X(0) - B1.x) / Y_hat(1);
//        C(1,1)= 2*(X(1) - B1.y) / Y_hat(1);
//        C(2,0)= 2*(X(0) - B1.x) / Y_hat(2);
//        C(2,1)= 2*(X(1) - B1.y) / Y_hat(2);

        // measurement covariance Q_gamma
        Q_gamma(0,0)= .05; // Q_gamma(1,1)= .1;  Q_gamma(2,2)= .1;

        // Residual covariance D is Q_(Y-Y_hat)
        D= ( C*P*C.transpose() + Q_gamma );

        // Correction propotional gain  K= P*C.transpose() * (C*P*C.transpose() + Q_gamma )^-1
        K= P*C.transpose() * D.reverse();

        // Estimation Step: estimated_state Xk+1/k+1 = Xk+1/k + Kk (Yk - Yk_hat)
        X = X + K*(Y-Y_hat);

        // Estimated_covariance_est: Pk+1/k+1 = ( I - Kk*Ck ) * Pk+1/k
        P= ( I - K*C ) * P;

        // publish the final pose ........................................................................................................
        geometry_msgs::Pose2D pose;
        pose.x = X(0);
        pose.y = X(1);
        pose.theta =  X(2);
        pose_pub.publish(pose);



        rate.sleep();
    }


}
