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

const int           gMaxPoints = 200;
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

char gRecordDirectory[100];

float eagleExposureTime = 0.2f;
float eagleGain = 0.6f;
float defaultExposureTime = 0.06f;
float defaultGain = 0.1f;
float exposureScale = 1.0f;
float gainScale = 1.0f;
int cpaUpdateRate = 1;
int gDelayVIOStart = 0;
int nIterationsForCPABeforeVISLAMInit = 0;

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

   if( gApplicationMode != VIO_ONLY )
      mvSRW_Writer_AddAccel( gWriter, timestamp, valX, valY, valZ );

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
      float32_t exposure = 0.0f, gain = 0.0f;
      /// In the starting we try to have CPA component converge to some parameter values thats more relevant to current scene than default values
      if( cpaframeId == 0 && nIterationsForCPABeforeVISLAMInit > 1 )
      {
         for( int i = 0; i < nIterationsForCPABeforeVISLAMInit - 1; ++i )
            mvCPA_AddFrame( cpa, (uint8_t*)curFrameBuffer, cameraConfig.memoryStride );
      }

      mvCPA_AddFrame( cpa, (uint8_t*)curFrameBuffer, cameraConfig.memoryStride );
      mvCPA_GetValues( cpa, &exposure, &gain );
      exposure *= exposureScale;
      gain *= gainScale;
      if( cpaframeId % cpaUpdateRate == 0 )
      {
         eagleCamera->setExposureAndGain( exposure, gain );
         if( gApplicationMode != VIO_ONLY )
            mvSRW_Writer_AddCameraSettings( gWriter, timestamp, gain, exposure, 0 );
      }
   }
   cpaframeId++;

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
//  Displays the usage help
//------------------------------------------------------------------------------
void printHelp()
{
   static const char* gHelp =
      "USAGE: mvVISLAM [-t arg] [-d arg] [-m arg] [-g arg] [-e arg] [-E arg] [-G arg]"
      "[-f arg] [-F arg] [-a] [-h] [-H]\n"
      "-t "
      "\t arg denotes how many seconds to run the app (default unlimited)\n"
      "-d "
      "\t arg denotes destination path for recording directory\n"
      "-m "
      "\t arg denotes which mode to run 0-vislam only , 1-capture only, 2-vislam and capture\n"
      "-g "
      "\t arg denotes gain value for non auto exposure mode valid values <0-1>\n"
      "-e "
      "\t arg denotes exposure value for non auto exposure mode valid values <0-1>\n"
      "-E "
      "\t arg denotes exposure value range/scale for auto exposure mode valid values <0-1>\n"
      "-G "
      "\t arg denotes gain value range/scale for auto exposure mode valid values <0-1>\n"
      "-f "
      "\t arg denotes auto exposure filter size default 10\n"
      "-F "
      "\t arg denotes update rate for auto exposure deault 1, every frame.\n"
      "-a"
      "\t enables auto exposure \n"
      "-H"
      "\t Print help information. \n"
      "-h"
      "\t Print help information.\n";
   printf( "%s\n", gHelp );
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
            printHelp();
            exit( 1 );
         case 'H':
            printHelp();
            exit( 1 );
         case '?':
            if( optopt == 't' || optopt == 'm' || optopt == 'd' ||
                optopt == 'g' || optopt == 'e' || optopt == 'E' ||
                optopt == 'G' || optopt == 'f' || optopt == 'F' )
               printf( "option -%c requires an argument\n", optopt );
         default:
            printf( "unknown argument\n" );
            printHelp();
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
         printHelp();
         exit( 1 );
      }
   }
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

         printf( "%d, %0.4f, %s, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f\n",
                 vioFrameId, viPose.timeAlignment, ErrorCodeToString( viPose.errorCode ).c_str(),
                 viPose.bodyPose.matrix[0][0], viPose.bodyPose.matrix[0][1], viPose.bodyPose.matrix[0][2], viPose.bodyPose.matrix[0][3],
                 viPose.bodyPose.matrix[1][0], viPose.bodyPose.matrix[1][1], viPose.bodyPose.matrix[1][2], viPose.bodyPose.matrix[1][3],
                 viPose.bodyPose.matrix[2][0], viPose.bodyPose.matrix[2][1], viPose.bodyPose.matrix[2][2], viPose.bodyPose.matrix[2][3],
                 viPose.velocity[0], viPose.velocity[1], viPose.velocity[2] );

      }

      if( gApplicationMode != VIO_ONLY )
         mvSRW_Writer_AddImage( gWriter, curFrame.timestamp, curFrame.buffer );
      vioFrameId++;
      gMemoryPool.releaseBlock( curFrame.buffer );
   }

   printf( "VISLAMWorkerFunc thread terminated\n" );
}

// -----------------------------------------------------------------------------
//  Application main
//-------------------------------------------------------------------------------
int main( int argc, char* argv[] )
{
   printf( "Starting mvVISLAM Sample Application\n" );

   if( argc > 1 )
   {
      parseCommandLine( argc, argv );
   }

   // Set up camera configuration
   cameraConfig.pixelWidth = 640;
   cameraConfig.pixelHeight = 480;

   //#define IMX377

#ifdef IMX377

    // hires camera (IMX377)
   cameraConfig.principalPoint[0] = 316.92;
   cameraConfig.principalPoint[1] = 239.50;

   cameraConfig.focalLength[0] = 197.209;
   cameraConfig.focalLength[1] = 197.209;

   cameraConfig.distortion[0] = 0.014323;
   cameraConfig.distortion[1] = -0.021431;
   cameraConfig.distortion[2] = 0.006077;
   cameraConfig.distortion[3] = -0.000954;
   cameraConfig.distortion[4] = 0;
   cameraConfig.distortion[5] = 0;
   cameraConfig.distortion[6] = 0;
   cameraConfig.distortion[7] = 0;
   cameraConfig.distortionModel = 10;

   float32_t readoutTime = 0.0287;

   // <Stateinit delta = "0.0171" ombc="1.0183190171 -1.0116207625 -1.3099902568" tbc="-0.074 0.013 -0.012"   /> <!--RS camera IMX377 10 deg downtilt -->

#else

    // tracking camera (OV 7251)
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


   float32_t tbc[3];
   float32_t ombc[3];

   //#define  DOWNTILTED_45_DEG

#if not defined (ISA_8x74_v7a)

   float32_t delta = 0.0026f;     // for Excelsior/8x96 for platform build 114 and higher (HAL3)

#ifdef DOWNTILTED_45_DEG // 45 deg down tilted camera on Excelsior

   tbc[0] = -0.03071f;
   tbc[1] = 0.00341f;
   tbc[2] = 0.00802f;

   ombc[0] = 0.61394f;
   ombc[1] = -0.61394f;
   ombc[2] = -1.48218f;

#else // down facing camera on Excelsior

   tbc[0] = 0.005f; // tbc here needs to be verified
   tbc[1] = 0.015f;
   tbc[2] = 0.0f;

   ombc[0] = 0.0f;
   ombc[1] = 0.0f;
   ombc[2] = -1.57f;

#endif

#else

   float32_t delta = -0.0068f;     // for Eagle/8x74 (after March 2016) and for Excelsior/8x96 for platform build 113 and lower (HAL1)

#ifdef DOWNTILTED_45_DEG // 45 deg down tilted camera on Eagle/8074

   tbc[0] = 0.00046f;
   tbc[1] = 0.01616f;
   tbc[2] = 0.00949f;

   ombc[0] = 0.61394f;
   ombc[1] = 0.61394f;
   ombc[2] = 1.48218f;

#else // down facing camera on Eagle/8074

   tbc[0] = 0.005f;
   tbc[1] = 0.0150f;
   tbc[2] = 0.0f;

   ombc[0] = 0.0f;
   ombc[1] = 0.0f;
   ombc[2] = 1.57f;

#endif

#endif

   float32_t std0Delta = 0.001f;   // firmware/driver upgrades may affect the time alignment

   float32_t std0Tbc[3];
   std0Tbc[0] = 0.005f;
   std0Tbc[1] = 0.005f;
   std0Tbc[2] = 0.005f;

   float32_t std0Ombc[3];
   std0Ombc[0] = 0.04f;
   std0Ombc[1] = 0.04f;
   std0Ombc[2] = 0.04f;

   float32_t accelMeasRange = 156.f;
   float32_t gyroMeasRange = 34.f;

   float32_t stdAccelMeasNoise = 0.316227766016838f; // sqrt(1e-1);
   float32_t stdGyroMeasNoise = 1e-2f; // sqrt(1e-4);

   float32_t stdCamNoise = 100.f;
   float32_t minStdPixelNoise = 0.5f;
   float32_t failHighPixelNoiseScaleFactor = 1.6651f;

   float32_t logDepthBootstrap = 0.f;
   bool useLogCameraHeight = false;
   float32_t logCameraHeightBootstrap = -3.22f;

   bool noInitWhenMoving = true;

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
   //BlurCameraParams::OutputFormatType defaultCameraOutFormat = BlurCameraParams::NV12_FORMAT; // IMX377
   cameraConfig.memoryStride = (defaultCameraOutFormat == BlurCameraParams::RAW_FORMAT) ?
      (cameraConfig.pixelWidth / 4) * 5 : cameraConfig.pixelWidth;
   float defaultFrameRate = 30.f;

   eagleCameraParams.pSize[0] = cameraConfig.pixelWidth;
   eagleCameraParams.pSize[1] = cameraConfig.pixelHeight;
   eagleCameraParams.frameRate = defaultFrameRate;
   eagleCameraParams.captureMode = BlurCameraParams::PREVIEW;
   eagleCameraParams.func = BlurCameraParams::CAM_FUNC_OPTIC_FLOW; // downward facing or 45 deg tilted
   //eagleCameraParams.func = BlurCameraParams::CAM_FUNC_HIRES; // IMX377
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
      return -1;
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

   shutdownApp = false;
   int elapsedSeconds = 0;
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

   // Wait until the worker thread is done
   workerThread.join();

   // Now initiate shutdown sequence
   shutdown();

   printf( "mvVISLAM finished\n" );

   return 0;
}
