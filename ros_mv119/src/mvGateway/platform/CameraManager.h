/***************************************************************************//**
@internal
   Copyright (c) 2017-2019 Qualcomm Technologies, Inc.
   All rights reserved.

   Redistribution and use in source and binary forms, with or without 
   modification, are permitted (subject to the limitations in the disclaimer 
   below) provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
     this list of conditions and the following disclaimer in the documentation 
     and/or other materials provided with the distribution.
   * Neither the name of Qualcomm Technologies, Inc. nor the names of its 
     contributors may be used to endorse or promote products derived from this 
     software without specific prior written permission.

   NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY 
   THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT 
   NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
   PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
*******************************************************************************/

#ifndef _CAMERA_MANAGER_H_
#define _CAMERA_MANAGER_H_

#include "camera.h"
#include "camera_parameters.h"

#include "CameraManagerConfig.h"
#include "CameraManagerHelper.h"
#include "mvCPA.h"
#include "mvSRW.h"

#include <list>
#include <mutex>
#include <condition_variable>


class CameraFrameListener
{
public:
    virtual void newCameraFrameReceived( int64_t, unsigned char* ) = 0;
    virtual ~CameraFrameListener() = 0;
};


class CameraManager : ICameraListener
{
public:

   CameraManager();
   virtual ~CameraManager();

   // initialize the camera manager with appropriate config
   bool init( CameraManagerConfig& cfg );

   // de-initialize the camera manager
   bool deinit();

   // starts the camera
   bool start();

   // stop the camera
   bool stop();

   // time stamp in nanoseconds
   int64_t getNextFrame( int64_t minFrameId, int64_t* timestamp, 
                         unsigned char* frameBuffer, 
                         unsigned char* frameBufferR = NULL,
                         CameraManagerConfig::CameraMangerOutputFormatType outputFormat = CameraManagerConfig::IMAGE_8_BIT_GRAY );

   // listener methods
   virtual void onError();
   virtual void onPreviewFrame( ICameraFrame* frame );
   virtual void onVideoFrame( ICameraFrame* frame );
   virtual void onPictureFrame( ICameraFrame* frame );

   void printCapabilities();
   void enableCPALogging( mvSRW_Writer * writer );
   void addNewListener( CameraFrameListener* listener );

   const CameraManagerConfig& getCameraConfig() const
   {
      return mCameraManagerCfg;
   }


private:
   struct CameraFrameType
   {
      CameraFrameType() : _frameId( -1 ), _timestamp( -1 ),
         _frame( nullptr ), _frameR( nullptr )
      {}

      CameraFrameType( int64_t frameId, int64_t timestamp,
                       ICameraFrame* frame, ICameraFrame* frameR = nullptr ) :
         _frameId( frameId ), _timestamp( timestamp ),
         _frame( frame ), _frameR( frameR )
      {
         if( _frame != nullptr )
            _frame->acquireRef();

         if( _frameR != nullptr )
            _frameR->acquireRef();
      }

      CameraFrameType( const CameraFrameType& ref ) :
         _frameId( ref._frameId ), _timestamp( ref._timestamp ),
         _frame( ref._frame ), _frameR( ref._frameR )
      {
         if( _frame != nullptr )
            _frame->acquireRef();

         if( _frameR != nullptr )
            _frameR->acquireRef();
      }

      CameraFrameType& operator=( const CameraFrameType& ref )
      {
         _frameId = ref._frameId;
         _timestamp = ref._timestamp;
         _frame = ref._frame;
         _frameR = ref._frameR;

         if( _frame != nullptr )
            _frame->acquireRef();

         if( _frameR != nullptr )
            _frameR->acquireRef();

         return *this;
      }

      ~CameraFrameType()
      {
         if( _frame != nullptr ) _frame->releaseRef();
         if( _frameR != nullptr ) _frameR->releaseRef();
      }

      int64_t getFrameId() const { return _frameId; };
      int64_t getTimestamp() const { return _timestamp; }
      ICameraFrame* getFrame() const { return _frame; }
      ICameraFrame* getFrameR() const { return _frameR; }

   private:
      int64_t _frameId;
      int64_t _timestamp;
      ICameraFrame* _frame;
      ICameraFrame* _frameR;
   };


   int initialize();
   int setParameters();
   void setExposureAndGain( float exposure, float gain );
   float getCameraExposureTime( ICameraFrame* frame );


   CameraManagerConfig mCameraManagerCfg;
   CameraParams mCameraParams;
   CameraCapabilities mCameraCaps;
   ICameraDevice* mCamera;
   int mCamId;
   size_t mFrameId;
   std::vector<CameraFrameListener*> mListeners;
   float mCameraExposureHist[2];

   std::mutex mFrameBufferMtx;
   std::condition_variable mHasNewFrameCV;
   std::list<CameraFrameType> mFrameBuffer;
   bool isRunning;

   mvCPA* mCPAObj;
   mvSRW_Writer* mSRWObj;
};

#endif
