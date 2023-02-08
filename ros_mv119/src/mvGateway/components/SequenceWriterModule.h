/***************************************************************************//**
@internal
   Copyright 2015 Qualcomm Technologies, Inc.  All rights reserved.
   Confidential & Proprietary.
*******************************************************************************/

#ifndef _SEQUENCE_WRITER_MODULE_H_
#define _SEQUENCE_WRITER_MODULE_H_

#include <thread>
#include <string>
#include <vector>
#include "mvSRW.h"
#include "CameraManager.h"
#include "ImuManager.h"

class SequenceWriterModule : public CameraFrameListener, public ImuFrameListener
{
public:
   struct SR_Configuration
   {
      int numCameras;
      std::string recordDirectory;
      bool isStereo;
      bool captureAttitude;
      SR_Configuration() : numCameras( 1 ), recordDirectory( "/home/linaro/Record" ), isStereo( false ), captureAttitude( false )
      {}
   };

   SequenceWriterModule();
   virtual ~SequenceWriterModule();

   /* intializes the sequence writer module with appropriate config */
   bool init( ImuManager* imuManager, CameraManager* cameraManager, SR_Configuration* srConfig );

   /* de intializes the sequence writer module */
   bool deinit();

   /* starts sequence writer processing */
   bool start();

   /* stops sequence writer processing */
   bool stop();

   //do nothing as sequence writer camera processing is slow. Dont want to do any processing in camera thread.
   void newCameraFrameReceived( int64_t timestamp, unsigned char * buffer )
   {};

   void newAccelDataReceived( float valX, float valY, float valZ, int64_t timestamp );
   void newGyroDataReceived( float valX, float valY, float valZ, int64_t timestamp );


private:
   SequenceWriterModule( const SequenceWriterModule& );
   SequenceWriterModule& operator=( const SequenceWriterModule& );

   void sequenceWriterProcessingMain();
   mvSRW_Writer*                 mMVSRWPtr;
   ImuManager*                   mImuManagerPtr;
   CameraManager*                mCameraManagerPtr;
   SR_Configuration              mSwConfig;
   sensor_attitude*              mPlatformAttitudeArray;
   mvAttitudeData*               mMvAttitudeArray;
   bool                          mInitialized;
   bool                          mRunning;
   std::thread                   mSwThread;
};
#endif
