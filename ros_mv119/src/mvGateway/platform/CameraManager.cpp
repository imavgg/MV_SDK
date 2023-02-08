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

#include "CameraManager.h"
#include <unistd.h>
#include <algorithm>
#include <chrono>

using namespace std;



CameraManager::CameraManager()
{
   mCamId = 0;
   mFrameId = 0;
   isRunning = false;
   mCPAObj = NULL;
   mSRWObj = NULL;
}



CameraManager::~CameraManager()
{}



void CameraManager::enableCPALogging( mvSRW_Writer* writer )
{
   mSRWObj = writer;
}


void CameraManager::onError()
{
   cout << "MV, ERROR, CameraManager onError!" << endl;
}



float CameraManager::getCameraExposureTime( ICameraFrame *frame )
{
#ifndef _HAL3_CAMERA_
   int exposureValue = mCameraExposureHist[1] * mCameraManagerCfg.maxExposureValue;
   if( exposureValue < mCameraManagerCfg.minExposureValue )
      exposureValue = mCameraManagerCfg.minExposureValue;

   return (mCameraManagerCfg.rowPeriodUs * exposureValue * 1000.0);
#else
   return mCameraParams.getFrameExposureTime( frame );
#endif
}



void CameraManager::setExposureAndGain( float exposure, float gain )
{
   int exposureValue;
   int gainValue;
   bool updateExposure = false;
   bool updateGain = false;

   if( exposure >= 0.f && exposure <= 1.f )
   {
      int prevExposureValue = mCameraManagerCfg.exposure * mCameraManagerCfg.maxExposureValue;
      if( prevExposureValue < mCameraManagerCfg.minExposureValue )
         prevExposureValue = mCameraManagerCfg.minExposureValue;

      exposureValue = exposure * mCameraManagerCfg.maxExposureValue;
      if( exposureValue < mCameraManagerCfg.minExposureValue )
         exposureValue = mCameraManagerCfg.minExposureValue;

      if( exposureValue != prevExposureValue )
         updateExposure = true;

      mCameraManagerCfg.exposure = exposure;
      mCameraExposureHist[1] = mCameraExposureHist[0];
      mCameraExposureHist[0] = exposure;
   }

   if( gain >= 0.f && gain <= 1.f )
   {
      int prevGainValue = mCameraManagerCfg.gain * mCameraManagerCfg.maxGainValue;
      if( prevGainValue < mCameraManagerCfg.minGainValue )
         prevGainValue = mCameraManagerCfg.minGainValue;

      gainValue = gain * mCameraManagerCfg.maxGainValue;
      if( gainValue < mCameraManagerCfg.minGainValue )
         gainValue = mCameraManagerCfg.minGainValue;

      if( gainValue != prevGainValue )
         updateGain = true;

      mCameraManagerCfg.gain = gain;
   }

   if( !updateExposure && !updateGain )
      return;

   if( updateExposure )
      mCameraParams.setManualExposure( exposureValue );

   if( updateGain )
      mCameraParams.setManualGain( gainValue );

   mCameraParams.commit();

   if( updateExposure )
      cout << "MV, INFO, set exposure " << exposureValue << endl;

   if( updateGain )
      cout << "MV, INFO, set gain " << gainValue << endl;
}



int CameraManager::setParameters()
{
   /* temp: using hard-coded values to test the api
   need to add a user interface or script to get the values to test*/
   //int pSizeIdx = 2;   // 640 , 480
   //int vSizeIdx = 2;   // 640 , 480
   int focusModeIdx = 3;
   int wbModeIdx = 2;
   int isoModeIdx = 5;   /// iso800
   int pFpsIdx = -1;
   int vFpsIdx = -1;
   int defaultPFps = 3; /// 30 fps
   int defaultVFps = 3; /// 30 fps
   mCameraCaps.pSizes = mCameraParams.getSupportedPreviewSizes();
   mCameraCaps.vSizes = mCameraParams.getSupportedVideoSizes();
   mCameraCaps.focusModes = mCameraParams.getSupportedFocusModes();
   mCameraCaps.wbModes = mCameraParams.getSupportedWhiteBalance();
   mCameraCaps.isoModes = mCameraParams.getSupportedISO();
   mCameraCaps.brightness = mCameraParams.getSupportedBrightness();
   mCameraCaps.sharpness = mCameraParams.getSupportedSharpness();
   mCameraCaps.contrast = mCameraParams.getSupportedContrast();
   mCameraCaps.previewFpsRanges = mCameraParams.getSupportedPreviewFpsRanges();
   mCameraCaps.videoFpsValues = mCameraParams.getSupportedVideoFps();

   mCameraExposureHist[0] = mCameraManagerCfg.exposure;
   mCameraExposureHist[1] = mCameraManagerCfg.exposure;

   ImageSize frameSize;
   if( mCameraManagerCfg.func == CameraManagerConfig::CAM_FUNC_STEREO )
      frameSize.width = mCameraManagerCfg.pSize[0] * 2;
   else
      frameSize.width = mCameraManagerCfg.pSize[0];

   frameSize.height = mCameraManagerCfg.pSize[1];

   cout << "MV, INFO, settings preview size " << frameSize.width << " & " << frameSize.height << endl;
   mCameraParams.setPreviewSize( frameSize );

   cout << "MV, INFO, settings video size " << frameSize.width << " & " << frameSize.height << endl;
   mCameraParams.setVideoSize( frameSize );

   if( mCameraManagerCfg.func == CameraManagerConfig::CAM_FUNC_HIRES )
   {
#ifndef _HAL3_CAMERA_
      cout << "MV, INFO, setting ISO mode: " << mCameraCaps.isoModes[isoModeIdx].c_str() << endl;
      mCameraParams.setISO( mCameraCaps.isoModes[isoModeIdx] );
      cout << "MV, INFO, setting focus mode: " << mCameraCaps.focusModes[focusModeIdx].c_str() << endl;
      mCameraParams.setFocusMode( mCameraCaps.focusModes[focusModeIdx] );
      cout << "MV, INFO, setting WB mode: " << mCameraCaps.wbModes[wbModeIdx].c_str() << endl;
      mCameraParams.setWhiteBalance( mCameraCaps.wbModes[wbModeIdx] );
#else
      cout << "MV, INFO, setting ISO mode: " << ISO_AUTO << endl;
      mCameraParams.setISO( ISO_AUTO );
      cout << "MV, INFO, setting focus mode: " << FOCUS_MODE_AUTO << endl;
      mCameraParams.setFocusMode( FOCUS_MODE_AUTO );
      cout << "MV, INFO, setting WB mode: " << WHITE_BALANCE_AUTO << endl;
      mCameraParams.setWhiteBalance( WHITE_BALANCE_AUTO );
#endif
   }
   if( mCameraCaps.previewFpsRanges.size() <= defaultPFps )
   {
      cout << "MV, INFO, default preview FPS index " << defaultPFps << "  greater than number of supported fps ranges " << mCameraCaps.previewFpsRanges.size() << " setting to " << mCameraCaps.previewFpsRanges.size() - 1 << " as default\n" << endl;
      defaultPFps = mCameraCaps.previewFpsRanges.size() - 1;
   }
   if( mCameraCaps.videoFpsValues.size() <= defaultVFps )
   {
      cout << "MV, INFO, default video FPS index " << defaultVFps << " greater than number of supported fps ranges " << mCameraCaps.videoFpsValues.size() << " setting to " << mCameraCaps.videoFpsValues.size() - 1 << " as default" << endl;
      defaultVFps = mCameraCaps.videoFpsValues.size() - 1;
   }
   for( int i = 0; i < mCameraCaps.previewFpsRanges.size(); ++i )
   {
      if( (mCameraCaps.previewFpsRanges[i].max) / 1000 == mCameraManagerCfg.frameRate )
      {
         pFpsIdx = i;
         if( (mCameraCaps.previewFpsRanges[i].min) / 1000 == mCameraManagerCfg.frameRate )
            break;
      }
   }
   for( int i = 0; i < mCameraCaps.videoFpsValues.size(); ++i )
   {
      if( mCameraCaps.videoFpsValues[i] == mCameraManagerCfg.frameRate )
      {
         vFpsIdx = i;
         break;
      }
   }
   if( pFpsIdx == -1 )
   {
      cout << "MV, INFO, couldn't find preview fps index for requested frame rate " << mCameraManagerCfg.frameRate << " setting default index " << defaultPFps << endl;
      pFpsIdx = defaultPFps;
   }
   if( vFpsIdx == -1 )
   {
      cout << "MV, INFO, couldn't find video fps index for requested frame rate " << mCameraManagerCfg.frameRate << " setting default index " << defaultVFps << endl;
      vFpsIdx = defaultVFps;
   }

   cout << "MV, INFO, setting preview fps range(idx " << pFpsIdx << " ): " << mCameraCaps.previewFpsRanges[pFpsIdx].min << " , " << mCameraCaps.previewFpsRanges[pFpsIdx].max << endl;
   mCameraParams.setPreviewFpsRange( mCameraCaps.previewFpsRanges[pFpsIdx] );
   cout << "MV, INFO, setting video fps (idx " << vFpsIdx << " ): " << mCameraCaps.videoFpsValues[vFpsIdx] << endl;
   mCameraParams.setVideoFPS( mCameraCaps.videoFpsValues[vFpsIdx] );

   if( mCameraManagerCfg.outputFormat == CameraManagerConfig::RAW_FORMAT )
   {
      cout << "MV, INFO, setting outputFormat RAW_FORMAT" << endl;
#ifndef _HAL3_CAMERA_
      mCameraParams.set( "preview-format", "bayer-rggb" );
      mCameraParams.set( "picture-format", "bayer-mipi-10gbrg" );
      mCameraParams.set( "raw-size", "640x480" );
#else
      mCameraParams.setPreviewFormat( FORMAT_RAW10 );
#endif
   }
   if( mCameraManagerCfg.func != CameraManagerConfig::CAM_FUNC_STEREO_RAW &&
       mCameraManagerCfg.outputFormat == CameraManagerConfig::NV12_FORMAT )
   {
      //cout << "MV, INFO, setting outputFormat nv12" << endl;
      //mCameraParams.set( "preview-format", "nv12" );
   }

   cout << "MV, INFO, set up params " << endl;

   int ret = mCameraParams.commit();
   cout << "MV, INFO, Committing params in setParameters " << endl;

   return ret;
}



int CameraManager::initialize()
{
   int rc;
   cout << "MV, INFO, Creating Camera Device instance with id:" << mCamId << endl;
   rc = ICameraDevice::createInstance( mCamId, &mCamera );
   if( rc != 0 )
   {
      cout << "MV, ERROR, could not open camera" << mCamId << endl;
      return rc;
   }
   mCamera->addListener( this );

   rc = mCameraParams.init( mCamera );
   if( rc != 0 )
   {
      cout << "MV, ERROR, failed to init parameters" << endl;
      ICameraDevice::deleteInstance( &mCamera );
   }

   return rc;
}



bool CameraManager::init( CameraManagerConfig& cfg )
{
   mCameraManagerCfg = cfg;
   int ret;
   {
      int n = getNumberOfCameras();

      cout << "MV, INFO, num_cameras = " << n << endl;

      if( n < 1 )
      {
         cout << "MV, ERROR, No cameras found" << endl;
         return false;
      }

      mCamId = -1;

      /* find camera based on function */
#ifndef _HAL3_CAMERA_
      for( int i = 0; i < n; i++ )
      {
         CameraInfo info;
         getCameraInfo( i, info );
         cout << "MV, INFO i=" << i << ", info.func=" << info.func << endl;
         if( info.func == mCameraManagerCfg.func )
         {
            mCamId = i;
         }
      }
#else
      switch( mCameraManagerCfg.func )
      {
         case CameraManagerConfig::CAM_FUNC_HIRES:
            mCamId = 0;
            break;
         case CameraManagerConfig::CAM_FUNC_OPTIC_FLOW:
            mCamId = 1;
            break;
         case CameraManagerConfig::CAM_FUNC_STEREO:
            mCamId = 2;
            break;
      }
#endif

      if( mCamId == -1 )
      {
         cout << "MV, ERROR, Camera not found" << endl;
         exit( 1 );
      }


      cout << "MV, INFO, initializing camera id= " << mCamId << endl;

      ret = initialize();
      if( ret != 0 )
      {
         cout << "MV, ERROR, initializing camera with id " << mCamId << " failed with err " << ret << endl;
         return false;
      }
   }

   ret = setParameters();
   if( ret != 0 )
   {
      cout << "MV, ERROR, setParameters camera failed with err " << ret << endl;
      return false;
   }

   if( mCameraManagerCfg.useCPA )
   {
      mCPAObj = mvCPA_Initialize( &mCameraManagerCfg.cpaConfig );
      if( mCPAObj == nullptr )
      {
         fprintf( stderr, "MV, ERROR, Cannot initialize CPA\n" );
         return false;
      }
   }

   return true;
}



bool CameraManager::deinit()
{
   bool ok = false;
   if( mCameraManagerCfg.useCPA )
      mvCPA_Deinitialize( mCPAObj );

   /* release camera device */
   ICameraDevice::deleteInstance( &mCamera );

   cout << "MV, INFO, Camera deinit done" << endl;

   return ok;
}



void CameraManager::printCapabilities()
{
   cout << "MV, INFO, Camera capabilities" << endl;
   mCameraCaps.pSizes = mCameraParams.getSupportedPreviewSizes();
   mCameraCaps.vSizes = mCameraParams.getSupportedVideoSizes();
   mCameraCaps.focusModes = mCameraParams.getSupportedFocusModes();
   mCameraCaps.wbModes = mCameraParams.getSupportedWhiteBalance();
   mCameraCaps.isoModes = mCameraParams.getSupportedISO();
   mCameraCaps.brightness = mCameraParams.getSupportedBrightness();
   mCameraCaps.sharpness = mCameraParams.getSupportedSharpness();
   mCameraCaps.contrast = mCameraParams.getSupportedContrast();
   mCameraCaps.previewFpsRanges = mCameraParams.getSupportedPreviewFpsRanges();
   mCameraCaps.videoFpsValues = mCameraParams.getSupportedVideoFps();

   cout << "MV, INFO, available preview sizes:" << endl;
   for( int i = 0; i < mCameraCaps.pSizes.size(); i++ )
   {
      cout << "MV, INFO, " << i << ": " << mCameraCaps.pSizes[i].width << " , " << mCameraCaps.pSizes[i].height << endl;
   }
   cout << "MV, INFO, available video sizes:" << endl;
   for( int i = 0; i < mCameraCaps.vSizes.size(); i++ )
   {
      cout << "MV, INFO, " << i << ": " << mCameraCaps.vSizes[i].width << " , " << mCameraCaps.vSizes[i].height << endl;
   }
   cout << "MV, INFO, available focus modes:" << endl;
   for( int i = 0; i < mCameraCaps.focusModes.size(); i++ )
   {
      cout << "MV, INFO, " << i << ": " << mCameraCaps.focusModes[i].c_str() << endl;
   }
   cout << "MV, INFO, available white balance modes:" << endl;
   for( int i = 0; i < mCameraCaps.wbModes.size(); i++ )
   {
      cout << "MV, INFO, " << i << ": " << mCameraCaps.wbModes[i].c_str() << endl;
   }
   cout << "MV, INFO, available ISO modes:" << endl;
   for( int i = 0; i < mCameraCaps.isoModes.size(); i++ )
   {
      cout << "MV, INFO, " << i << ": " << mCameraCaps.isoModes[i].c_str() << endl;
   }
   cout << "MV, INFO, available brightness values:" << endl;
   cout << "MV, INFO, min=" << mCameraCaps.brightness.min << ", max=" << mCameraCaps.brightness.max << ", step=" << mCameraCaps.brightness.step << endl;

   cout << "MV, INFO, available sharpness values:" << endl;
   cout << "MV, INFO, min=" << mCameraCaps.sharpness.min << ", max=" << mCameraCaps.sharpness.max << ", step=" << mCameraCaps.sharpness.step << endl;

   cout << "MV, INFO, available contrast values:" << endl;
   cout << "MV, INFO, min=" << mCameraCaps.contrast.min << ", max=" << mCameraCaps.contrast.max << ", step=" << mCameraCaps.contrast.step << endl;

   cout << "MV, INFO, available preview fps ranges:" << endl;
   for( int i = 0; i < mCameraCaps.previewFpsRanges.size(); i++ )
   {
      cout << "MV, INFO, i:" << i << " min=" << mCameraCaps.previewFpsRanges[i].min << ", max=" << mCameraCaps.previewFpsRanges[i].max << endl;
   }

   cout << "MV, INFO, available video fps values:" << endl;
   for( int i = 0; i < mCameraCaps.videoFpsValues.size(); i++ )
   {
      cout << "MV, INFO, i:" << i << " " << mCameraCaps.videoFpsValues[i] << endl;
   }
}



void CameraManager::addNewListener( CameraFrameListener* listener )
{
   mListeners.push_back( listener );
}



void CameraManager::onPreviewFrame( ICameraFrame *frame )
{
   //cout << "MV, INFO, got a new preview frame " << endl;

   if( frame == nullptr || !isRunning )
   {
      cout << "MV, INFO, Frame is null or camera has been stopped" << endl;
      return;
   }

#ifndef _HAL3_CAMERA_
   // HAL3 timestamp measures the end of exposure of first row
   float correction = -getCameraExposureTime( frame ) / 2.0f;
#else
   // HAL3 timestamp measures the start of exposure of first row
   float correction = getCameraExposureTime( frame ) / 2.0f;
#endif

   CameraFrameType curFrame( mFrameId, frame->timeStamp + (int64_t)correction, frame );

   for( auto listener : mListeners )
      listener->newCameraFrameReceived( curFrame.getTimestamp(), curFrame.getFrame()->data );

   // Protected by mFrameBufferMtx
   {
      unique_lock<mutex> lck( mFrameBufferMtx );

      if( mFrameBuffer.size() == mCameraManagerCfg.maxFrameBufferSize )
      {
         mFrameBuffer.erase( mFrameBuffer.begin() );
      }

      // push back the current frame into the buffer
      mFrameBuffer.push_back( curFrame );
      //printf( "Number of buffers: %d\n", mFrameBuffer.size() );
   }
   mHasNewFrameCV.notify_all();

   float32_t exposure = mCameraManagerCfg.exposure;
   float32_t gain = mCameraManagerCfg.gain;

   // CPA
   /// In tests we notice that there is a delay of 3 frames for the camera parameters to get applied. Hence
   /// we wait for 3 frames before computing and applying new cpa values. hence default cpaUpdateRate is 4
   if( mCameraManagerCfg.useCPA && mFrameId % mCameraManagerCfg.cpaUpdateRate == 0 )
   {
      // auto t1 = chrono::steady_clock::now();
      mvCPA_AddFrame( mCPAObj, curFrame.getFrame()->data, mCameraManagerCfg.pStride );
      mvCPA_GetValues( mCPAObj, &exposure, &gain );
      // auto t2 = chrono::steady_clock::now();
      //cout << chrono::duration_cast<chrono::microseconds>(t2 - t1).count() << endl;
   }

   setExposureAndGain( exposure, gain );
   if( mSRWObj )
   {
      mvSRW_Writer_AddCameraSettings( mSRWObj, curFrame.getTimestamp(), gain, exposure, 0 );
   }

   //cout << "MV, INFO, received frame id: " << mFrameId << endl;
   mFrameId++;
}



void CameraManager::onVideoFrame( ICameraFrame *frame )
{
   cout << "onVideoFrame is not implemented" << endl;
}



void CameraManager::onPictureFrame( ICameraFrame * frame )
{
   cout << "onPictureFrame is not implemented" << endl;
}



int64_t CameraManager::getNextFrame( int64_t minFrameId, int64_t* timestamp, 
                                     unsigned char* frameBuffer, unsigned char* frameBufferR, 
                                     CameraManagerConfig::CameraMangerOutputFormatType outputFormat )
{
   if( outputFormat == CameraManagerConfig::IMAGE_16_BIT_GRAY )
   {
      if( !(mCameraManagerCfg.func == CameraManagerConfig::CAM_FUNC_STEREO_RAW || 
            mCameraManagerCfg.outputFormat == CameraManagerConfig::RAW_FORMAT) )
      {
         cout << "MV, ERROR, Requested format ( outputFormat =  " << outputFormat << " )is not supported for this configuration " << endl;
         return -1;
      }
   }

   // Protected by mFrameBufferMtx
   CameraFrameType nextFrame;
   {
      unique_lock<mutex> lck( mFrameBufferMtx );

      list<CameraFrameType>::iterator nextFrameIt = mFrameBuffer.end();
      while( isRunning )
      {
         nextFrameIt = find_if( mFrameBuffer.begin(), mFrameBuffer.end(),
                                     [&]( const CameraFrameType& frame )
                                     {
                                        return frame.getFrameId() > minFrameId;
                                     } );

         if( nextFrameIt != mFrameBuffer.end() )
            break;

         mHasNewFrameCV.wait( lck );
      }

      if( nextFrameIt == mFrameBuffer.end() )
         return -1;

      nextFrame = *nextFrameIt;
      mFrameBuffer.erase( nextFrameIt );
   }

   int64_t frameId = -1;
   auto nBytes = CameraManagerHelper::getImageSizeInBytes( mCameraManagerCfg.pSize[0], mCameraManagerCfg.pSize[1] );
   if( mCameraManagerCfg.func == CameraManagerConfig::CAM_FUNC_STEREO && frameBufferR != NULL )
   {
      //printf( "Split w=%d  h=%d nBytes=%d sz=%d\n", mCameraManagerCfg.pSize[0], mCameraManagerCfg.pSize[1], nBytes, nextFrame.getFrame()->size ); fflush( stdout );
      for( int i = 0; i < mCameraManagerCfg.pSize[1]; ++i )
      {
         unsigned char* srcL = nextFrame.getFrame()->data + i * mCameraManagerCfg.pSize[0] * 2;
         unsigned char* srcR = srcL + mCameraManagerCfg.pSize[0];

         memcpy( frameBuffer + i*mCameraManagerCfg.pSize[0], srcL, mCameraManagerCfg.pSize[0] );
         memcpy( frameBufferR + i*mCameraManagerCfg.pSize[0], srcR, mCameraManagerCfg.pSize[0] );
      }
      frameId = nextFrame.getFrameId();
   }
   else if( mCameraManagerCfg.func == CameraManagerConfig::CAM_FUNC_STEREO_RAW && frameBufferR != NULL )
   {
      if( outputFormat == CameraManagerConfig::IMAGE_8_BIT_GRAY )
      {
         CameraManagerHelper::convertAtlRawToGrayScale( nextFrame.getFrame()->data, frameBuffer, mCameraManagerCfg.pSize[0], mCameraManagerCfg.pSize[1] );
         CameraManagerHelper::convertAtlRawToGrayScale( nextFrame.getFrameR()->data, frameBufferR, mCameraManagerCfg.pSize[0], mCameraManagerCfg.pSize[1] );
      }
      else if( outputFormat == CameraManagerConfig::IMAGE_16_BIT_GRAY )
      {
         CameraManagerHelper::convertAtlRawToGrayScale16( nextFrame.getFrame()->data, frameBuffer, mCameraManagerCfg.pSize[0], mCameraManagerCfg.pSize[1] );
         CameraManagerHelper::convertAtlRawToGrayScale16( nextFrame.getFrameR()->data, frameBufferR, mCameraManagerCfg.pSize[0], mCameraManagerCfg.pSize[1] );
      }
      else
      {
         cout << "MV, ERROR, Requested format ( outputFormat =  " << outputFormat << " )is not supported for this configuration " << endl;
         return false;
      }
      frameId = nextFrame.getFrameId();
   }
   else if( frameBuffer != NULL )
   {
      if( mCameraManagerCfg.outputFormat == CameraManagerConfig::RAW_FORMAT )
      {
         if( outputFormat == CameraManagerConfig::IMAGE_8_BIT_GRAY )
         {
            CameraManagerHelper::convertAtlRawToGrayScale( nextFrame.getFrame()->data, frameBuffer, mCameraManagerCfg.pSize[0], mCameraManagerCfg.pSize[1] );
         }
         else if( outputFormat == CameraManagerConfig::IMAGE_16_BIT_GRAY )
         {
            CameraManagerHelper::convertAtlRawToGrayScale16( nextFrame.getFrame()->data, frameBuffer, mCameraManagerCfg.pSize[0], mCameraManagerCfg.pSize[1] );
         }
         else
         {
            cout << "MV, ERROR, Requested format ( outputFormat =  " << outputFormat << " )is not supported for this configuration " << endl;
            return false;
         }
      }
      else
      {
         memcpy( frameBuffer, nextFrame.getFrame()->data, nBytes );
      }
      frameId = nextFrame.getFrameId();
   }

   if( frameId != -1 )
   {
      *timestamp = nextFrame.getTimestamp();
   }

   return frameId;
}



bool CameraManager::start()
{
   unique_lock<mutex> lck( mFrameBufferMtx );

   int ret = 0;
   if( mCameraManagerCfg.captureMode == CameraManagerConfig::PREVIEW )
   {
      cout << "MV, INFO, start preview" << endl;
      isRunning = true;
      ret = mCamera->startPreview();
      if( 0 != ret )
      {
         cout << "MV, ERROR, start preview failed" << ret << endl;
      }
   }
   else if( mCameraManagerCfg.captureMode == CameraManagerConfig::VIDEO )
   {
      cout << "MV, INFO, start recording" << endl;
      isRunning = true;
      ret = mCamera->startRecording();
      if( 0 != ret )
      {
         cout << "MV, ERROR, start recording failed" << ret << endl;
      }
   }

   if( 0 != ret )
      return false;

   float tmpExposure = mCameraManagerCfg.exposure;
   float tmpGain = mCameraManagerCfg.gain;

   // setExposureAndGain only updates if different values
   mCameraManagerCfg.exposure = 0.f;
   mCameraManagerCfg.gain = 0.f;

   setExposureAndGain( tmpExposure, tmpGain );

#if defined ( _HAL3_CAMERA_ )
   if( (mCameraManagerCfg.func == CameraManagerConfig::CAM_FUNC_STEREO) )
   {
      mCameraParams.setVerticalFlip( true );
      mCameraParams.setHorizontalMirror( true );
      printf( "Setting Vertical Flip and Horizontal Mirror bit in sensor \n" );
      // mCameraParams.commit();
   }
#endif

   mCameraParams.setSharpness( 0 );
   mCameraParams.commit();

   return true;
}



bool CameraManager::stop()
{
   unique_lock<mutex> lck( mFrameBufferMtx );

   if( mCameraManagerCfg.captureMode == CameraManagerConfig::PREVIEW )
   {
      cout << "MV, INFO, stop preview" << endl;

      isRunning = false;
      mCamera->stopPreview();

      cout << "MV, INFO, stop preview done" << endl;
   }
   else if( mCameraManagerCfg.captureMode == CameraManagerConfig::VIDEO )
   {
      cout << "MV, INFO, stop recording" << endl;
      isRunning = false;
      mCamera->stopRecording();
      cout << "MV, INFO, stop recording done" << endl;
   }

   cout << "MV, INFO, releasing frames" << endl;
   mFrameBuffer.clear();

   mHasNewFrameCV.notify_all();

   return true;
}



CameraFrameListener::~CameraFrameListener()
{}

