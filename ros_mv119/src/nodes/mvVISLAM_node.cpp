/****************************************************************************
 *   Copyright (c) 2016 Ramakrishna Kintada. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include "SnapdragonRosNodeVislam.hpp"
#include "mvVISLAM_main.h"

#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>



Snapdragon::RosNode::Vislam::Vislam( ros::NodeHandle nh ) : nh_(nh)
{
  pub_vislam_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("vislam/pose",1);
  pub_vislam_odometry_ = nh_.advertise<nav_msgs::Odometry>("vislam/odometry",1);
  vislam_initialized_ = false;
  thread_started_ = false;
  thread_stop_ = false;
  // sleep here so tf buffer can get populated
  ros::Duration(1).sleep(); // sleep for 1 second
}

Snapdragon::RosNode::Vislam::~Vislam()
{
  Stop();
}

int32_t Snapdragon::RosNode::Vislam::Initialize()
{ 
  vislam_initialized_ = true;
  return 0;
}

int32_t Snapdragon::RosNode::Vislam::Start() {
// start vislam processing thread.
  if( !thread_started_ ) {
    thread_started_ = true;
    vislam_process_thread_ = std::thread( &Snapdragon::RosNode::Vislam::ThreadMain, this );
  }
  else {
    ROS_WARN_STREAM( "Snapdragon::RosNodeVislam::Start() VISLAM Thread already running." );
  }
  return 0;
}

int32_t Snapdragon::RosNode::Vislam::Stop() {
  if( thread_started_ ) {
    thread_stop_ = true;
    if( vislam_process_thread_.joinable() ) {
      vislam_process_thread_.join();
    }
  }
  return 0;
}



void Snapdragon::RosNode::Vislam::ThreadMain() {

//   if( vislam_man.Initialize( param, vislamParams ) != 0  ) {
//     ROS_WARN_STREAM( "Snapdragon::RosNodeVislam::VislamThreadMain: Error initializing the VISLAM Manager " );
//     thread_started_ = false;
//     return;
//   }

// // start the VISLAM processing.
//   if( vislam_man.Start() != 0 ) {
//     ROS_WARN_STREAM( "Snapdragon::RosNodeVislam::VislamThreadMain: Error Starting the VISLAM manager" );
//     thread_started_ = false;
//     return;
//   }



  while( !thread_stop_ ) {


    // add thread for mvvilsam main function
    std::thread mvvislam_thread(mvVISLAM_main);

    // if( vislam_ret == 0 ) {
         mvPose6DRT  pose = vi_pose.bodyPose;
        float x = pose.matrix[0][3]; 
        float y = pose.matrix[1][3]; 
        float z = pose.matrix[2][3];
        // printf("%d, x=%0.4f, y=%0.4f, z=%0.4f \n",
        //    vislamPose.poseQuality, x,y,z);
      //check if the pose quality is good.  If not do not publish the data.
      if( vi_pose.poseQuality != MV_TRACKING_STATE_FAILED  && 
          vi_pose.poseQuality != MV_TRACKING_STATE_INITIALIZING ) {
          // Publish Pose Data
          int64_t timestamp_ns= vi_pose.timeAlignment;
          int64_t vislamFrameId = vioFrameId;
          PublishVislamData( vi_pose, vislamFrameId, timestamp_ns );
          
      }
        char szResult[256];
        PoseToString( vi_pose.poseQuality, szResult, sizeof( szResult ) );
        printf( szResult );

        printf( "return : %0.4f, %s, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f\n",
        vi_pose.timeAlignment, ErrorCodeToString( vi_pose.errorCode ).c_str(),
        vi_pose.bodyPose.matrix[0][0], vi_pose.bodyPose.matrix[0][1], vi_pose.bodyPose.matrix[0][2], vi_pose.bodyPose.matrix[0][3],
        vi_pose.bodyPose.matrix[1][0], vi_pose.bodyPose.matrix[1][1], vi_pose.bodyPose.matrix[1][2], vi_pose.bodyPose.matrix[1][3],
        vi_pose.bodyPose.matrix[2][0], vi_pose.bodyPose.matrix[2][1], vi_pose.bodyPose.matrix[2][2], vi_pose.bodyPose.matrix[2][3],
        vi_pose.velocity[0], vi_pose.velocity[1], vi_pose.velocity[2] );

    // }
    // else {
    //   ROS_WARN_STREAM( "Snapdragon::RosNodeVislam::VislamThreadMain: Warning Getting Pose Information" );
    // }
        mvvislam_thread.join();

  }
  thread_started_ = false;
  // the thread is shutting down. Stop the vislam Manager.
//   vislam_man.Stop();


  ROS_INFO_STREAM( "Snapdragon::RosNodeVislam::VislamThreadMain: Exising VISLAM Thread" );
  return;
}

// publish pose estimate to ros
int32_t Snapdragon::RosNode::Vislam::PublishVislamData( mvVISLAMPose& vislamPose, int64_t vislamFrameId, uint64_t timestamp_ns  ) {
  geometry_msgs::PoseStamped pose_msg;
  ros::Time frame_time;
  frame_time.sec = (int32_t)(timestamp_ns/1000000000UL);
  frame_time.nsec = (int32_t)(timestamp_ns % 1000000000UL);
  pose_msg.header.frame_id = "vislam";
  pose_msg.header.stamp = frame_time;
  pose_msg.header.seq = vislamFrameId;

  // translate vislam pose to ROS pose
  tf2::Matrix3x3 R(
    vislamPose.bodyPose.matrix[0][0],
    vislamPose.bodyPose.matrix[0][1],
    vislamPose.bodyPose.matrix[0][2],
    vislamPose.bodyPose.matrix[1][0],
    vislamPose.bodyPose.matrix[1][1],
    vislamPose.bodyPose.matrix[1][2],
    vislamPose.bodyPose.matrix[2][0],
    vislamPose.bodyPose.matrix[2][1],
    vislamPose.bodyPose.matrix[2][2]); 
  tf2::Quaternion q;
  R.getRotation(q);
  pose_msg.pose.position.x = vislamPose.bodyPose.matrix[0][3];
  pose_msg.pose.position.y = vislamPose.bodyPose.matrix[1][3];
  pose_msg.pose.position.z = vislamPose.bodyPose.matrix[2][3];
  pose_msg.pose.orientation.x = q.getX();
  pose_msg.pose.orientation.y = q.getY();
  pose_msg.pose.orientation.z = q.getZ();
  pose_msg.pose.orientation.w = q.getW();
  pub_vislam_pose_.publish(pose_msg);

  //publish the odometry message.(vel , ang vel)
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = frame_time;
  odom_msg.header.frame_id = "vislam";
  odom_msg.pose.pose = pose_msg.pose;
  odom_msg.twist.twist.linear.x = vislamPose.velocity[0];
  odom_msg.twist.twist.linear.y = vislamPose.velocity[1];
  odom_msg.twist.twist.linear.z = vislamPose.velocity[2];
  odom_msg.twist.twist.angular.x = vislamPose.angularVelocity[0];
  odom_msg.twist.twist.angular.y = vislamPose.angularVelocity[1];
  odom_msg.twist.twist.angular.z = vislamPose.angularVelocity[2];

  //set the error covariance for the pose.
  for( int16_t i = 0; i < 6; i++ ) {
    for( int16_t j = 0; j < 6; j++ ) {
      odom_msg.pose.covariance[ i*6 + j ] = vislamPose.errCovPose[i][j];
    }
  }
  pub_vislam_odometry_.publish(odom_msg); 

  // compute transforms
  std::vector<geometry_msgs::TransformStamped> transforms;
  geometry_msgs::TransformStamped transform;
  transform.transform.translation.x = pose_msg.pose.position.x;
  transform.transform.translation.y = pose_msg.pose.position.y;
  transform.transform.translation.z = pose_msg.pose.position.z;
  transform.transform.rotation.x = pose_msg.pose.orientation.x;
  transform.transform.rotation.y = pose_msg.pose.orientation.y;
  transform.transform.rotation.z = pose_msg.pose.orientation.z;
  transform.transform.rotation.w = pose_msg.pose.orientation.w;
  transform.child_frame_id = "base_link_vislam";
  transform.header.frame_id = "vislam";
  transform.header.stamp = frame_time;

  // collect transforms
  transforms.push_back(transform);

  // broadcast transforms
  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(transforms);     
}



 void Snapdragon::RosNode::Vislam::PoseToString( MV_TRACKING_STATE poseQuality, char *pszResult, int nResultLen )
{
   memset( pszResult, 0, nResultLen );

   strncat( pszResult, "mvTrackingPose: ", nResultLen );

   // Pose Quality
   //strncat( pszResult, "Pose Quality ", nResultLen );
   switch( poseQuality )
   {
      case MV_TRACKING_STATE_FAILED:
         strncat( pszResult, "NOT TRACKING", nResultLen );
         break;
      case MV_TRACKING_STATE_INITIALIZING:
         strncat( pszResult, "INITIALIZING", nResultLen );
         break;
      case MV_TRACKING_STATE_HIGH_QUALITY:
         strncat( pszResult, "HIGH_QUALITY", nResultLen );
         break;
      case MV_TRACKING_STATE_LOW_QUALITY:
         strncat( pszResult, "LOW_QUALITY", nResultLen );
         break;
      default:
         strncat( pszResult, "Unknown", nResultLen );
         break;
   }
   strncat( pszResult, "\n", nResultLen );
}