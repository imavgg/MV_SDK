/***************************************************************************//**
@internal
   Copyright 2014 Qualcomm Technologies, Inc.  All rights reserved.
   Confidential & Proprietary.
*******************************************************************************/

#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>
#include <mvVISLAM.h>
#include "mvSRW.h"
#include "mvVISLAMDemoData.h"
#include "mvMemoryPool.h"
#include "mvCPA.h"
#include <EagleImu.h>
#include <EagleCamera.h>
#include <list>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <linux/limits.h>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
// #include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

//-----------------------------------------------------------------------------
//  Types
//-----------------------------------------------------------------------------
struct CameraFrameType
{
   int64_t timestamp;
   unsigned char *buffer;
};

enum APP_MODE
{
   VIO_ONLY,
   CAPTURE_ONLY,
   VIO_AND_CAPTURE
};


//-----------------------------------------------------------------------------
//  Globals
//-----------------------------------------------------------------------------

mvVISLAM*                   gVISLAMTracker = NULL;
mvSRW_Writer*               gWriter = NULL;
std::mutex                  gFrameBufferMtx;
std::condition_variable     gHasNewFrameCV;
std::list<CameraFrameType>  gFrameBuffer;

const int gMaxFrameBufferSize = 4;
const int gVGAFrameNBytes = 307200;

MemoryPool gMemoryPool;
APP_MODE gApplicationMode = VIO_ONLY;

const int           gMaxPoints = 500; //200
mvVISLAMMapPoint    gPointCloud[gMaxPoints];

// Maintains total distance travelled by VIO since last reset
float distanceTravelled = 0.f;

static bool shutdownApp;
int         gTimeToRunTheApp = 0;

EagleImu*               eagleImu;
EagleCamera*            eagleCamera;
mvCameraConfiguration   cameraConfig;

mvCPA*      cpa = NULL;
bool        useCpa = false;
uint32_t    cpaFilterSize = 1;

// char gRecordDirectory[100];
char gRecordDirectory[250];

float eagleExposureTime = 0.2f;
float eagleGain = 0.6f;
float defaultExposureTime = 0.06f;
float defaultGain = 0.1f;
float exposureScale = 1.0f;
float gainScale = 1.0f;
int cpaUpdateRate = 1;
int gDelayVIOStart = 0;
int nIterationsForCPABeforeVISLAMInit = 0;

sensor_msgs::Image ros_image;
sensor_msgs::Imu imu_msg;
ros::Publisher   pub_vislam_pose_, pub_vislam_odometry_, pub_vislam_pc_,pub_vislam_pc2_, pub_vislam_pcl_, pub_cam_, imu_pub_;
// -----------------------------------------------------------------------------
//     Create a string from an MV_TRACKING_STATE
//-------------------------------------------------------------------------------
static void PoseToString( MV_TRACKING_STATE poseQuality, char *pszResult, int nResultLen )
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

// -----------------------------------------------------------------------------
//     Create a string from an errorCode
//-------------------------------------------------------------------------------
std::string ErrorCodeToString( uint32_t errorCode )
{
   std::string result;

   if( errorCode == 0 ) return("no error");
   else
   {
      if( CHECK_BIT( errorCode, 0 ) )
         result = result + "Reset: cov not pos definite ";
      if( CHECK_BIT( errorCode, 1 ) )
         result = result + "Reset: IMU exceeded range ";
      if( CHECK_BIT( errorCode, 2 ) )
         result = result + "Reset: IMU bandwidth too low ";
      if( CHECK_BIT( errorCode, 3 ) )
         result = result + "Reset: not stat at init ";
      if( CHECK_BIT( errorCode, 4 ) )
         result = result + "Reset: no features for x seconds ";
      if( CHECK_BIT( errorCode, 5 ) )
         result = result + "Reset: insufficient constraints from features ";
      if( CHECK_BIT( errorCode, 6 ) )
         result = result + "Reset: failed to add new features ";
      if( CHECK_BIT( errorCode, 7 ) )
         result = result + "Reset: exceeded instant velocity uncertainty ";
      if( CHECK_BIT( errorCode, 8 ) )
         result = result + "Reset: exceeded velocity uncertainty over window ";
      if( CHECK_BIT( errorCode, 10 ) )
         result = result + "dropped IMU samples ";
      if( CHECK_BIT( errorCode, 11 ) )
         result = result + "check intrinsic camera cal ";
      if( CHECK_BIT( errorCode, 12 ) )
         result = result + "too few good features to init ";
      if( CHECK_BIT( errorCode, 13 ) )
         result = result + "dropped frame ";
      if( CHECK_BIT( errorCode, 14 ) )
         result = result + "dropped GPS velocity sample ";
      if( CHECK_BIT( errorCode, 15 ) )
         result = result + "sensor measurements w/ uninitialized time stamps or uncertainty ";
      if (CHECK_BIT(errorCode, 16))
          result = result + "missing IMU measurements in prop to most recent IMU step ";
   }

   return result;
}

// -----------------------------------------------------------------------------
//  Called when acceleration data is available
// -----------------------------------------------------------------------------
void accelCallback( float valX, float valY, float valZ, int64_t timestamp )
{
   //acceleration values to the filter
   // printf( "Accel: %lli %.5f %.5f %.5f \n", timestamp, valX, valY, valZ );
   if( gApplicationMode != CAPTURE_ONLY )
      mvVISLAM_AddAccel( gVISLAMTracker,
                         timestamp,
                         valX,
                         valY,
                         valZ );
   //Pass Accelerometer data to the SequenceWriter object. 
   if( gApplicationMode != VIO_ONLY )
      mvSRW_Writer_AddAccel( gWriter, timestamp, valX, valY, valZ );



   // imu publish to ros

   imu_msg.linear_acceleration.x = valX;
   imu_msg.linear_acceleration.y = valY;
   imu_msg.linear_acceleration.z = valZ;
   imu_msg.header.frame_id = "vislam_imu";
   imu_msg.header.stamp = ros::Time (timestamp);
}

// -----------------------------------------------------------------------------
//  Called when gyroscope data available
// -----------------------------------------------------------------------------
void gyroCallback( float valX, float valY, float valZ, int64_t timestamp )
{
   //gyro values to the filter
   //printf( "Gyro: %lli %.5f %.5f %.5f \n", timestamp, valX, valY, valZ );
   if( gApplicationMode != CAPTURE_ONLY )
      mvVISLAM_AddGyro( gVISLAMTracker,
                        timestamp,
                        valX,
                        valY,
                        valZ );

   if( gApplicationMode != VIO_ONLY )
      mvSRW_Writer_AddGyro( gWriter, timestamp, valX, valY, valZ );

   imu_msg.angular_velocity.x = valX;
   imu_msg.angular_velocity.y = valY;
   imu_msg.angular_velocity.z = valZ;

   imu_pub_.publish(imu_msg);



}


void camToros(sensor_msgs::Image  image, unsigned char *curFrameBuffer)
{
   image.encoding = std::string("mono8");
   /*yuv_map_ == MONO*/
   sensor_msgs::fillImage( image, 
                           sensor_msgs::image_encodings::MONO8,
                           cameraConfig.principalPoint[0],
                           cameraConfig.principalPoint[1],
                           cameraConfig.principalPoint[1],
                           (uint8_t*)curFrameBuffer);

   /* RAW FORMAT*/
   /*  if(snap_cam_param_.camera_config.cam_format == Snapdragon::CameraFormat::RAW_FORMAT){
            unsigned int image_size_bytes = snap_cam_param_.camera_config.pixel_height *
               snap_cam_param_.camera_config.memory_stride;
            image->data.resize(image_size_bytes);
            for(unsigned int i=0; i*4 < image_size_bytes; ++i){
               memcpy(&(image->data[i*4]),
               reinterpret_cast<uint8_t*>(image_buffer)+i*5, 4);
            }
            image->encoding = std::string("mono8");
            image->step = snap_cam_param_.camera_config.pixel_width;
    */
   // unsigned int image_size_bytes = cameraConfig.principalPoint[0] *  cameraConfig.principalPoint[1];
   // image.data.resize(image_size_bytes);
   // for(unsigned int i=0; i*4 < image_size_bytes; ++i)
   // {
   //    // copying data of size 4 bytes starting from the address reinterpret_cast<uint8_t*>(curFrameBuffer)+i*5 to another location
   //    memcpy(&(image.data[i*4]), reinterpret_cast<uint8_t*>(curFrameBuffer)+i*5, 4);
   //  }


   image.step = cameraConfig.principalPoint[0];
   image.is_bigendian = 0;
   image.width = cameraConfig.principalPoint[0] ;
   image.height = cameraConfig.principalPoint[1] ;
   image.step = cameraConfig.principalPoint[0] ;
   pub_cam_.publish(image);
   
}
// -----------------------------------------------------------------------------
// Called when a new camera frame is available
// -----------------------------------------------------------------------------
void cameraCallback( int64_t timestamp, unsigned char *curFrameBuffer )
{
   CameraFrameType curFrame;
   float correction = -1000 * (eagleCamera->getCameraExposureTime() / 2.0f);
   curFrame.timestamp = timestamp + (int64_t)correction;
   curFrame.buffer = (unsigned char *)gMemoryPool.acquireBlock();
   if( curFrame.buffer != nullptr )
      memcpy( curFrame.buffer, curFrameBuffer, gVGAFrameNBytes );

   std::unique_lock<std::mutex> lck( gFrameBufferMtx );
   // if buffer size full erase the former image frames
   if( gFrameBuffer.size() == gMaxFrameBufferSize )
   {
      printf( "reached frame buffer size of %d, removing old frame", gMaxFrameBufferSize );
      //gMemoryPool.releaseBlock( gFrameBuffer.begin()->buffer );
      gFrameBuffer.erase( gFrameBuffer.begin() );
   }
   // push back the current frame into the buffer
   gFrameBuffer.push_back( curFrame );
   gHasNewFrameCV.notify_all();

   // CPA
   static int cpaframeId = 0;
   /// In tests we notice that there is a delay of 3 frames for the camera parameters to get applied. Hence
   /// we wait for 3 frames before computing and applying new cpa values.
   if( cpa && cpaframeId % 4 == 0 )
   {
      printf("camera callback CPA\n");
      float32_t exposure = 0.0f, gain = 0.0f;
      /// In the starting we try to have CPA component converge to some parameter values thats more relevant to current scene than default values
      if( cpaframeId == 0 && nIterationsForCPABeforeVISLAMInit > 1 )
      {
         for( int i = 0; i < nIterationsForCPABeforeVISLAMInit - 1; ++i )
            mvCPA_AddFrame( cpa, (uint8_t*)curFrameBuffer, cameraConfig.memoryStride );
      }
      //Add image to adjust exposure and gain parameters
      mvCPA_AddFrame( cpa, (uint8_t*)curFrameBuffer, cameraConfig.memoryStride );
      mvCPA_GetValues( cpa, &exposure, &gain );
      exposure *= exposureScale;
      gain *= gainScale;
      if( cpaframeId % cpaUpdateRate == 0 )
      {
         eagleCamera->setExposureAndGain( exposure, gain );
         if( gApplicationMode != VIO_ONLY )
         //Pass CameraSettings data to the SequenceWriter object. 
            mvSRW_Writer_AddCameraSettings( gWriter, timestamp, gain, exposure, 0 );
      }
   }
   cpaframeId++;

   camToros(ros_image,curFrameBuffer);

}



//------------------------------------------------------------------------------
//  Handler for trapping Ctrl+C
//------------------------------------------------------------------------------

void sigIntFunc( int s )
{
   printf( "Caught signal %d\n", s );
   shutdownApp = true;

   // Now signal the CV so that the processing thread can terminate
   gHasNewFrameCV.notify_all();
}

//------------------------------------------------------------------------------
//  Terminates the application
//------------------------------------------------------------------------------
void shutdown()
{
   printf( "Shutting down...\n" );

   printf( "Stopping camera ..." );
   eagleImu->stopCapturing();
   if( eagleCamera->stop() )
   {
      printf( "successful\n" );
   }
   else
   {
      printf( "failed\n" );
   }
   if( gWriter )
   {
      mvSRW_Writer_Deinitialize( gWriter );
   }

   if( eagleCamera->deinit() )
   {
      printf( "Camera deinit() succesful\n" );
   }
   else
   {
      printf( "Camera deinit() failed\n" );
   }
   delete eagleCamera;

   printf( "De-initialising sensors\n" );
   eagleImu->deinit();
   delete eagleImu;


   if( gVISLAMTracker )
   {
      mvVISLAM_Deinitialize( gVISLAMTracker );
      printf( "mvVISLAM_Deinitialize() done.\n" );
   }

   printf( "Distance Travelled %.5f \n", distanceTravelled );

   if( cpa )
   {
      printf( "deinitialising CPA\n" );
      mvCPA_Deinitialize( cpa );
   }
}

//------------------------------------------------------------------------------
//  Parses the command line and sets various globals
//------------------------------------------------------------------------------
void parseCommandLine( int argc, char * argv[] )
{
   memset( &gRecordDirectory[0], 0, sizeof( gRecordDirectory ) );

   size_t dir_str_len = 0;
   int c, mode;
   bool exposureSet = false;
   bool gainSet = false;
   while( (c = getopt( argc, argv, "t:d:m:g:e:E:G:f:F:a::Hh" )) != -1 )
   {
      switch( c )
      {
         case 't':
            gTimeToRunTheApp = atoi( optarg );
            break;
         case 'm':
            mode = atoi( optarg );
            if( mode == 0 )
            {
               gApplicationMode = VIO_ONLY;
               printf( "mode vio\n" );
            }
            else if( mode == 1 )
            {
               gApplicationMode = CAPTURE_ONLY;
               printf( "mode capture\n" );
            }
            else if( mode == 2 )
            {
               gApplicationMode = VIO_AND_CAPTURE;
               printf( "mode vio and capture\n" );
            }
            break;
         case 'd':
            dir_str_len = sizeof( gRecordDirectory );
            strncpy( gRecordDirectory, optarg, dir_str_len );
            if( gRecordDirectory[dir_str_len] != '\0' )
            {
               printf( "Directory string too long for buffer\n" );
               exit( 1 );
            }
            break;
         case 'g':
            eagleGain = atof( optarg );
            if( eagleGain < 0.0f || eagleGain > 1 )
               eagleGain = defaultGain;
            gainSet = true;
            break;
         case 'e':
            eagleExposureTime = atof( optarg );
            if( eagleExposureTime < 0.0f )
               eagleExposureTime = defaultExposureTime;
            exposureSet = true;
            break;
         case 'E':
            exposureScale = atof( optarg );
            if( exposureScale < 0.0f || exposureScale > 1.0f )
               exposureScale = 1.0f;
            break;
         case 'G':
            gainScale = atof( optarg );
            if( gainScale < 0.0f || gainScale > 1.0f )
               gainScale = 1.0f;
            break;
         case 'a':
            useCpa = true;
            printf( "CPA Will use Automatic exposure and gain control \n" );
            break;
         case 'f':
            cpaFilterSize = atoi( optarg );
            //if (cpaFilterSize < 0)
            //    cpaFilterSize = 10;
            //break;
         case 'F':
            cpaUpdateRate = atoi( optarg );
            if( cpaUpdateRate < 1 )
               cpaUpdateRate = 1;
            printf( "Setting CPA update rate : %d\n", cpaUpdateRate );
            break;
         case 'h':
            // printHelp();
            exit( 1 );
         case 'H':
            // printHelp();
            exit( 1 );
         case '?':
            if( optopt == 't' || optopt == 'm' || optopt == 'd' ||
                optopt == 'g' || optopt == 'e' || optopt == 'E' ||
                optopt == 'G' || optopt == 'f' || optopt == 'F' )
               printf( "option -%c requires an argument\n", optopt );
         default:
            printf( "unknown argument\n" );
            // printHelp();
            exit( 1 );
      }
   }
   if( useCpa )
      gDelayVIOStart = 30;  //delay starting of VIO by 30 frames (1sec at 30fps)
   else
   {
      if( !exposureSet )
         eagleExposureTime = defaultExposureTime;
      if( !gainSet )
         eagleGain = defaultGain;

   }

   // Validate the recording directory if capture option is selected
   if( gApplicationMode != VIO_ONLY )
   {
      // See if the file name is defined
      if( strlen( gRecordDirectory ) == 0 )
      {
         printf( "Error: Missing capture directory.\n" );
         printf( "Please provide a directory name for saving capture data\n" );
         // printHelp();
         exit( 1 );
      }
   }
}

//------------------------------------------------------------------------------
//  Publish VISLAM data to ROS
//------------------------------------------------------------------------------
void PublishVislamData( mvVISLAMPose& vislamPose, std::vector<mvVISLAMMapPoint> &vio_points, int64_t vislamFrameId, uint64_t timestamp_ns  ) 
{

   // publish pose
  geometry_msgs::PoseStamped pose_msg;
  ros::Time frame_time;
  frame_time.sec = (int32_t)(timestamp_ns/1000000000UL);
  frame_time.nsec = (int32_t)(timestamp_ns % 1000000000UL);
  pose_msg.header.frame_id = "vislam";
  pose_msg.header.stamp = frame_time;
  pose_msg.header.seq = vislamFrameId;

  // translate vislam pose to ROS pose
  //Rotation
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
  pose_msg.pose.orientation.x = q.getX();
  pose_msg.pose.orientation.y = q.getY();
  pose_msg.pose.orientation.z = q.getZ();
  pose_msg.pose.orientation.w = q.getW();

   //Translation
  pose_msg.pose.position.x = vislamPose.bodyPose.matrix[0][3];
  pose_msg.pose.position.y = vislamPose.bodyPose.matrix[1][3];
  pose_msg.pose.position.z = vislamPose.bodyPose.matrix[2][3];
  pub_vislam_pose_.publish(pose_msg);

  //publish the odometry message.(linear vel and angular vel)
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


  // publish the pointcloud of vislam
  sensor_msgs::PointCloud cloud_msg;
  cloud_msg.points.reserve(vio_points.size());
  cloud_msg.header.frame_id = "vislam";
  sensor_msgs::ChannelFloat32 quality_channel;
  quality_channel.name = "point_quality";
  quality_channel.values.reserve(vio_points.size());

  for (auto it = vio_points.cbegin(); it != vio_points.cend(); ++it) {

    geometry_msgs::Point32 cloud_pt;
    /*id	Unique ID for map point.
      tsf :3D location in spatial frame in meters.
      p_tsf:	Error covariance for tsf.
      depth:	Depth of map point from camera in meters.
      depthErrorStdDev:	Depth error standard deviation in meters. */
   // tsf:  	3D location in spatial frame in meters.
    cloud_pt.x = it->tsf[0];
    cloud_pt.y = it->tsf[1];
    cloud_pt.z = it->tsf[2];
    cloud_msg.points.push_back(cloud_pt);

   //  printf("depth of point: %i = %i (m)\n", it->id,it->depth); 

    quality_channel.values.push_back(it->pointQuality);
  }
  
  pub_vislam_pc_.publish(cloud_msg);

   sensor_msgs::PointCloud2 cloud2_msg;
   sensor_msgs::convertPointCloudToPointCloud2 	(cloud_msg, cloud2_msg);

   pub_vislam_pc2_.publish(cloud2_msg);


   // publish pcl::PointXYZRGB points : toDo (can use pcl function)
   // pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;   
   // pub_vislam_pcl_.publish(pcl_cloud);

}

//------------------------------------------------------------------------------
//  Thread that processes the VIO data when a new camera frame is available
//------------------------------------------------------------------------------
void VISLAMWorkerFunc( mvVISLAM * tracker )
{
   while( !shutdownApp )
   {
      {
         std::unique_lock<std::mutex> lck( gFrameBufferMtx );
         while( gFrameBuffer.empty() && !shutdownApp )
         {
            gHasNewFrameCV.wait( lck );
            // We have finished waiting. This could either be due arrival
            // of a new camera frame OR shutdown request
            if( shutdownApp )
            {
               printf( "Got app shutdown request\n" );
               break;
            }
         }
      }

      // We do have camera data to process. But if application shutdown
      // was requested, then bail out
      if( shutdownApp )
      {
         printf( "Received shutdown\n" );
         break;
      }

      gFrameBufferMtx.lock();
      CameraFrameType curFrame = gFrameBuffer.front();
      gFrameBuffer.erase( gFrameBuffer.begin() );
      gFrameBufferMtx.unlock();
      static int vioFrameId = 0;

      if( gApplicationMode != CAPTURE_ONLY && vioFrameId >= gDelayVIOStart )
      {
         mvVISLAM_AddImage( tracker, curFrame.timestamp, curFrame.buffer );
         mvVISLAMPose viPose = mvVISLAM_GetPose( tracker );
         char szResult[256];
         PoseToString( viPose.poseQuality, szResult, sizeof( szResult ) );
         printf( szResult );

         // if point cloud is in good quality then push
         if( viPose.poseQuality != MV_TRACKING_STATE_FAILED  && 
          viPose.poseQuality != MV_TRACKING_STATE_INITIALIZING ) {


            printf( "%d, %0.4f, %s, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f\n",
                 vioFrameId, viPose.timeAlignment, ErrorCodeToString( viPose.errorCode ).c_str(),
                 viPose.bodyPose.matrix[0][0], viPose.bodyPose.matrix[0][1], viPose.bodyPose.matrix[0][2], viPose.bodyPose.matrix[0][3],
                 viPose.bodyPose.matrix[1][0], viPose.bodyPose.matrix[1][1], viPose.bodyPose.matrix[1][2], viPose.bodyPose.matrix[1][3],
                 viPose.bodyPose.matrix[2][0], viPose.bodyPose.matrix[2][1], viPose.bodyPose.matrix[2][2], viPose.bodyPose.matrix[2][3],
                 viPose.velocity[0], viPose.velocity[1], viPose.velocity[2] );

            int64_t timestamp_ns= viPose.timeAlignment;
            int64_t vislamFrameId = vioFrameId;

            //max number of points requrested.
            int num_points = mvVISLAM_HasUpdatedPointCloud(tracker);
            // current map points of array structure
            std::vector<mvVISLAMMapPoint> map_points(num_points, {0});
            // get number of points and push to map_points, if there is new map points, then update and estimated.
            int num_received = mvVISLAM_GetPointCloud(tracker, map_points.data(), num_points);
            // Publish Pose and cloud Data
            PublishVislamData( viPose, map_points, vislamFrameId, timestamp_ns );
            

         }

         if( gApplicationMode != VIO_ONLY )
            mvSRW_Writer_AddImage( gWriter, curFrame.timestamp, curFrame.buffer );
         vioFrameId++;
         gMemoryPool.releaseBlock( curFrame.buffer );
      }
   }

}


// -----------------------------------------------------------------------------
//  Application main
//-------------------------------------------------------------------------------
int main( int argc, char* argv[] )
{
   printf( "Starting mvVISLAM Sample Application\n" );

   ros::init(argc, argv, "mvVISLAM_publisher");

   ros::NodeHandle nh_;
   pub_vislam_pose_= nh_.advertise<geometry_msgs::PoseStamped>("vislam/pose",10);
   pub_vislam_odometry_=nh_.advertise<nav_msgs::Odometry>("vislam/odometry",10);
   // pub_vislam_pc_ = nh_.advertise<sensor_msgs::PointCloud>("vislam/pointcloud", 1000);
   pub_vislam_pc2_ = nh_.advertise<sensor_msgs::PointCloud2>("vislam/pointcloud2", 1000);
   pub_cam_ = nh_.advertise<sensor_msgs::Image>("/image", 10);
   imu_pub_ = nh_.advertise<sensor_msgs::Imu>("vislam/imu", 10);


   if( argc > 1 )
   {
      parseCommandLine( argc, argv );
   }

   // Set up camera configuration
   cameraConfig.pixelWidth = 640;
   cameraConfig.pixelHeight = 480;

   // Set the camera configuration (IMX377 or ov7251)
   //#define IMX377

   #ifdef IMX377
      printf("Use imx777 camera..............\n");

      // hires camera (IMX377)
      // cameraConfig.principalPoint[0] = 316.92;
      // cameraConfig.principalPoint[1] = 239.50;

      // cameraConfig.focalLength[0] = 197.209;
      // cameraConfig.focalLength[1] = 197.209;

      // cameraConfig.distortion[0] = 0.014323;
      // cameraConfig.distortion[1] = -0.021431;
      // cameraConfig.distortion[2] = 0.006077;
      // cameraConfig.distortion[3] = -0.000954;
      // cameraConfig.distortion[4] = 0;
      // cameraConfig.distortion[5] = 0;
      // cameraConfig.distortion[6] = 0;
      // cameraConfig.distortion[7] = 0;
      // cameraConfig.distortionModel = 10;

      // float32_t readoutTime = 0.0287;

      // <Stateinit delta = "0.0171" ombc="1.0183190171 -1.0116207625 -1.3099902568" tbc="-0.074 0.013 -0.012"   /> <!--RS camera IMX377 10 deg downtilt -->

   #else

      // tracking camera (OV 7251)
      printf("Use OV 7251 camera..............\n");

      cameraConfig.principalPoint[0] = 320;
      cameraConfig.principalPoint[1] = 240;

      cameraConfig.focalLength[0] = 275;
      cameraConfig.focalLength[1] = 275;

      cameraConfig.distortion[0] = 0.003908;
      cameraConfig.distortion[1] = -0.009574;
      cameraConfig.distortion[2] = 0.010173;
      cameraConfig.distortion[3] = -0.003329;
      cameraConfig.distortion[4] = 0;
      cameraConfig.distortion[5] = 0;
      cameraConfig.distortion[6] = 0;
      cameraConfig.distortion[7] = 0;
      cameraConfig.distortionModel = 10;

      float32_t readoutTime = 0;  // 0 for global shutter
   #endif

      // Camera to IMU frame
      float32_t tbc[3]; // position of the camera frame origin with respect to the IMU frame origin, written in the IMU coordinate frame, in meters.
      float32_t ombc[3]; //orientation that transforms a vector in the camera frame to the IMU frame, converted to axis-angle representation (three components), in radians.

      //#define  DOWNTILTED_45_DEG

   #if not defined (ISA_8x74_v7a)

      // Camera, IMU time misalignment 
      float32_t delta = 0.0026f;     // for Excelsior/8x96 for platform build 114 and higher (HAL3)

      #ifdef DOWNTILTED_45_DEG // 45 deg down tilted camera on Excelsior
         printf("Use down 45 tilted camera..............\n");

         tbc[0] = -0.03071f;
         tbc[1] = 0.00341f;
         tbc[2] = 0.00802f;

         ombc[0] = 0.61394f;
         ombc[1] = -0.61394f;
         ombc[2] = -1.48218f;

      #else // down facing camera on Excelsior
         printf("Use downward camera..............\n");


         // tbc[0] = 0.005f; // tbc here needs to be verified: camera to body translation
         // tbc[1] = 0.015f;
         // tbc[2] = 0.0f;

         // ombc[0] = 0.0f;
         // ombc[1] = 0.0f;
         // ombc[2] = -1.57f;
         tbc[0] = -0.03071f;
         tbc[1] = 0.00341f;
         tbc[2] = 0.00802f;

         ombc[0] = 0.61394f;
         ombc[1] = -0.61394f;
         ombc[2] = -1.48218f;

      #endif

   #endif

      printf("tbc, ombc= [%f %f %f],[%f %f %f] \n", tbc[0],tbc[1],tbc[2],ombc[0],ombc[1],ombc[2]);
      //Default values

      float32_t std0Delta = 0.001f;   // firmware/driver upgrades may affect the time alignment

      float32_t std0Tbc[3];
      std0Tbc[0] = 0.005f;
      std0Tbc[1] = 0.005f;
      std0Tbc[2] = 0.005f;

      float32_t std0Ombc[3];
      std0Ombc[0] = 0.04f;
      std0Ombc[1] = 0.04f;
      std0Ombc[2] = 0.04f;

      float32_t accelMeasRange = 156.f;//
      float32_t gyroMeasRange = 34.f;

      float32_t stdAccelMeasNoise = 0.316227766016838f; // sqrt(1e-1);
      float32_t stdGyroMeasNoise = 1e-2f; // sqrt(1e-4);

      float32_t stdCamNoise = 100.f;
      float32_t minStdPixelNoise = 0.5f;
      float32_t failHighPixelNoiseScaleFactor = 1.6651f;

      float32_t logDepthBootstrap = 0.f;
      bool useLogCameraHeight = false;
      float32_t logCameraHeightBootstrap = -3.22f; //Downard 
      // allows to enable or disable initialization when moving. T
      bool noInitWhenMoving = true; // need static
      // bool noInitWhenMoving = false;


      float32_t limitedIMUbWtrigger = 35.f;

      std::string staticMaskFileName = "";

      float32_t gpsImuTimeAlignment = 0.0f;

      float32_t tba[3];
      tba[0] = tba[1] = tba[2] = 0.f;

      bool mapping = true;


      gMemoryPool.resetBlockSize( gVGAFrameNBytes );

      eagleCamera = new EagleCamera();
      BlurCameraParams eagleCameraParams;

      BlurCameraParams::OutputFormatType defaultCameraOutFormat = BlurCameraParams::RAW_FORMAT; // was NV12_FORMAT but on 8096, cannot set gain and exposure time
      // BlurCameraParams::OutputFormatType defaultCameraOutFormat = BlurCameraParams::NV12_FORMAT; // IMX377

      cameraConfig.memoryStride = (defaultCameraOutFormat == BlurCameraParams::RAW_FORMAT) ?
         (cameraConfig.pixelWidth / 4) * 5 : cameraConfig.pixelWidth;
      float defaultFrameRate = 30.f;

      eagleCameraParams.pSize[0] = cameraConfig.pixelWidth;
      eagleCameraParams.pSize[1] = cameraConfig.pixelHeight;
      eagleCameraParams.frameRate = defaultFrameRate;
      eagleCameraParams.captureMode = BlurCameraParams::PREVIEW;

      eagleCameraParams.func = BlurCameraParams::CAM_FUNC_OPTIC_FLOW; // downward facing or 45 deg tilted
      // eagleCameraParams.func = BlurCameraParams::CAM_FUNC_HIRES; // IMX377
      //eagleCameraParams.func = BlurCameraParams::CAM_FUNC_LEFT_SENSOR; // forward facing (or 45 deg tilted)

      eagleCameraParams.exposure = eagleExposureTime;
      eagleCameraParams.gain = eagleGain;
      eagleCameraParams.outputFormat = defaultCameraOutFormat;
      eagleCamera->setCaptureParams( eagleCameraParams );
      
      if( useCpa )
      {
         printf( "CPA Initializing with %f exposure, %f gain\n", eagleCameraParams.exposure, eagleCameraParams.gain );

         mvCPA_Configuration cpaConfig;
         cpaConfig.width = cameraConfig.pixelWidth;
         cpaConfig.height = cameraConfig.pixelHeight;
         cpaConfig.format = (defaultCameraOutFormat == BlurCameraParams::RAW_FORMAT) ? MVCPA_FORMAT_RAW10 : MVCPA_FORMAT_GRAY8;

         //  MVCPA_FORMAT_GRAY8: 8-bit grayscale format.
         //  MVCPA_FORMAT_RAW10: Android 10-bit raw format.
         //  MVCPA_FORMAT_RAW12: Android 12-bit raw format.

         //DK: Change cpaType to 1 if you want new approach
         cpaConfig.cpaType = MVCPA_MODE_COST;
         cpaConfig.legacyCost.startExposure = eagleCameraParams.exposure;
         cpaConfig.legacyCost.startGain = eagleCameraParams.gain;
         cpaConfig.legacyCost.filterSize = cpaFilterSize;
         cpaConfig.legacyCost.exposureCost = 1.0f;
         cpaConfig.legacyCost.gainCost = 0.3333f;

         cpa = mvCPA_Initialize( &cpaConfig );
         if( cpa == NULL )
         {
            printf( "Error initializing CPA\n" );
            return -1;
         }
      }

      if( gApplicationMode != VIO_ONLY )
      {
         mvMonoCameraInit cameraInit;
         cameraInit.name = "vioCamera";
         cameraInit.width = cameraConfig.pixelWidth;
         cameraInit.height = cameraConfig.pixelHeight;

         //Initialize with one mono camera
         gWriter = mvSRW_Writer_Initialize( gRecordDirectory, &cameraInit, NULL );
         if( gWriter == NULL )
         {
            printf( "Error initializing SRW writer\n" );
            return -1;
         }

         //Write camera parameters into file
         mvSRW_Writer_AddCameraParameters( gWriter, cameraInit.name,
                                          &cameraConfig );
      }


    // MV: Initialize the mvVISLAM tracker
         gVISLAMTracker = mvVISLAM_Initialize(
            &cameraConfig, readoutTime,
            tbc, ombc, delta,
            std0Tbc, std0Ombc, std0Delta,
            accelMeasRange, gyroMeasRange,
            stdAccelMeasNoise, stdGyroMeasNoise,
            stdCamNoise, minStdPixelNoise, failHighPixelNoiseScaleFactor,
            logDepthBootstrap, useLogCameraHeight, logCameraHeightBootstrap,
            noInitWhenMoving, limitedIMUbWtrigger,
            staticMaskFileName.c_str(),
            gpsImuTimeAlignment, tba,
            mapping);

         if( gVISLAMTracker == NULL )
         {
            printf( "Error initializing VISLAM\n" );
            return -1;
         }

         if( !eagleCamera->init() )
         {
            printf( "Error in camera.init()!\n" );
         }
         eagleCamera->addCallback( cameraCallback );
         
         std::thread workerThread( VISLAMWorkerFunc, gVISLAMTracker );
         printf( "Starting Sensors!\n" );
         eagleImu = new EagleImu();
         eagleImu->init();
         eagleImu->addAccelCallback( accelCallback );
         eagleImu->addGyroCallback( gyroCallback );

         if( !eagleImu->start() )
         {
            printf( "Error in sensor start!\n" );
            return -1;
         }

         printf( "Starting Camera!\n" );
         if( !eagleCamera->start() )
         {
            printf( "Error in camera start!\n" );
            return -1;
         }

         struct sigaction sigIntHandler;
         sigIntHandler.sa_handler = sigIntFunc;
         sigemptyset( &sigIntHandler.sa_mask );
         sigIntHandler.sa_flags = 0;
         sigaction( SIGINT, &sigIntHandler, NULL );

        
         //Run until shutdown is reuested
         printf( "Running mvVISLAM application. Press Ctrl+C to stop\n" );

      // while ROS open:
      ros::Rate loop_rate(1000);

      while (nh_.ok()) {
         printf("start ROS\n");

         shutdownApp = false;
         int elapsedSeconds = 0;

         // if app not shutdown
         while( !shutdownApp )
         {
            usleep( 1000000 );
            elapsedSeconds++;
            // Check to see if we are running in the timed capture mode
            // If so, then stop if necessary
            if( gTimeToRunTheApp > 0 )
            {
               if( elapsedSeconds >= gTimeToRunTheApp )
               {
                  // Finished runing
                  printf( "Finished running for %d seconds\n", elapsedSeconds );
                  shutdownApp = true;
                  // Now signal the CV so that the processing thread can terminate
                  gHasNewFrameCV.notify_all();

                  // Break out of this loop
                  break;
               }
            }
            printf( "[%d] Running (total: %d)...\n", elapsedSeconds, gTimeToRunTheApp );
         }

         // Now initiate shutdown sequence
         workerThread.join();

         shutdown();

         printf( "mvVISLAM finished\n" );


         // ROS will close at the end for completely turn off others method.
         ros::spinOnce();
         loop_rate.sleep();
         printf( "ROS finished\n" );

      }

        



}
