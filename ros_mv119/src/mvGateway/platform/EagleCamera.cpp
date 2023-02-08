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
#include <time.h>
#include "EagleCamera.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cstdint> 
const float rowPeriodUs = 19.3333;

EagleCamera::EagleCamera()
{
   callback = NULL;
   camId = 0;
   clockOffset = 0;
}

EagleCamera::~EagleCamera()
{}

void EagleCamera::onError()
{
   printf( "camera error!\n" );
}

void convertAtlRawToGrayScale( ICameraFrame *frame, int width, int height )
{
   int destSize = width * height;
   unsigned char* imageBuf = frame->data;
   for( int i = 0; 4 * i < destSize; ++i )
   {
      memcpy( reinterpret_cast<uint8_t*>(imageBuf) + i * 4, reinterpret_cast<uint8_t*>(frame->data) + i * 5, 4 );
   }
}

void convertAtlRawToGrayScale16( ICameraFrame *frame, int width, int height )
{
   int nPixels = width * height;
   int destSize = nPixels * 2;

   unsigned char* imageBuf = frame->data;
   uint16_t pL = 0, p0H = 0, p1H = 0, p2H = 0, p3H = 0, mask = 0x0003;
   for( int i = 0; 4 * i < nPixels; i += 4 )
   {
      p0H = *(reinterpret_cast<uint8_t*>(frame->data) + i * 5 + 0);
      p1H = *(reinterpret_cast<uint8_t*>(frame->data) + i * 5 + 1);
      p2H = *(reinterpret_cast<uint8_t*>(frame->data) + i * 5 + 2);
      p3H = *(reinterpret_cast<uint8_t*>(frame->data) + i * 5 + 3);
      pL = *(reinterpret_cast<uint8_t*>(frame->data) + i * 5 + 4);
      *(reinterpret_cast<uint16_t*>(imageBuf) + i * 4 + 0) = (uint16_t)((p0H << 2) | ((pL >> 6) & mask));
      *(reinterpret_cast<uint16_t*>(imageBuf) + i * 4 + 1) = (uint16_t)((p1H << 2) | ((pL >> 4) & mask));
      *(reinterpret_cast<uint16_t*>(imageBuf) + i * 4 + 2) = (uint16_t)((p2H << 2) | ((pL >> 2) & mask));
      *(reinterpret_cast<uint16_t*>(imageBuf) + i * 4 + 3) = (uint16_t)((p3H << 2) | ((pL >> 0) & mask));
   }
}

int64_t getMonotonicTime()
{
   struct timespec t;
   clock_gettime( CLOCK_MONOTONIC, &t );
   uint64_t timeNanoSecMonotonic = t.tv_sec * 1000000000ULL + t.tv_nsec;
   return (int64_t)timeNanoSecMonotonic;
}

int64_t getRealTime()
{
   struct timespec t;
   clock_gettime( CLOCK_REALTIME, &t );
   uint64_t timeNanoSecRealTime = t.tv_sec * 1000000000ULL + t.tv_nsec;
   return (int64_t)timeNanoSecRealTime;
}

int64_t getTimeEpoch()
{
   time_t now = time( 0 );
   uint64_t timeNanoSecEpoch = now * 1000000000ULL;
   return (int64_t)timeNanoSecEpoch;
}

double getDspClock()
{
   int64_t qdspTicks = 0;

   static const char qdspTimerTickPath[] = "/sys/kernel/boot_adsp/qdsp_qtimer";
   char qdspTicksStr[20] = "";
   static const double clockFreq = 1 / 19.2;
   FILE* qdspClockfp = fopen( qdspTimerTickPath, "r" );
   if( qdspClockfp != nullptr )
   {
      fread( qdspTicksStr, 16, 1, qdspClockfp );
      qdspTicks = strtoll( qdspTicksStr, 0, 16 );
      fclose( qdspClockfp );
   }
   return qdspTicks*clockFreq*1e3;
}

double getArchClock()
{
   const char archTimerTickPath[] = "/sys/kernel/boot_adsp/arch_qtimer";
   char archTicksStr[20] = "";
   static const double clockFreq = 1 / 19.2;
   FILE* archClockfp = fopen( archTimerTickPath, "r" );
   if( archClockfp != nullptr )
      fread( archTicksStr, 16, 1, archClockfp );

   int64_t archTicks = strtoll( archTicksStr, 0, 16 );
   fclose( archClockfp );
   return archTicks*clockFreq*1e3;
}

//Currently we assume camera uses monotonic clock
void EagleCamera::findClocksOffsetForCamera()
{
   int64_t dspClock = (int64_t)getDspClock();
   int64_t monotonicClock = getMonotonicTime();
   clockOffset = dspClock - monotonicClock;
   printf( "findClocksOffsetForCamera dspClock = %lld, monotonicClock=%lld, clockOffset=%lld \n ", dspClock, monotonicClock, clockOffset );
}

float EagleCamera::getCameraExposureTime()
{
   // return exposureValue in us; 

#if defined (ISA_8x74_v7a)
   float exposureValue = MIN_EXPOSURE_VALUE + cameraExposureHist[1] * (MAX_EXPOSURE_VALUE - MIN_EXPOSURE_VALUE);
   return (rowPeriodUs * exposureValue);
#else
    // minus sign because time stamp is at beginning of exposure, not at SOF
   return -exposureValueHAL3 / 1000;
   //return 0;
#endif
}

//#define PRINT_CLOCKS
void EagleCamera::onPreviewFrame( ICameraFrame *frame )
{
   static uint32_t countP = 0;

   if( eagleCaptureParams.outputFormat == BlurCameraParams::RAW_FORMAT )
   {
      convertAtlRawToGrayScale( frame, eagleCaptureParams.pSize[0], eagleCaptureParams.pSize[1] );
   }
#if defined (ISA_8x74_v7a)
   exposureValueHAL3 = -1;
#else
   exposureValueHAL3 = atlParams.getFrameExposureTime( frame );
#endif
   callback( frame->timeStamp + clockOffset, frame->data );

#ifdef PRINT_CLOCKS
   printf( "Cam Frame timestamp=%lld, monotonic=%lld, realtime=%lld, timeEpoch=%lld, timeDSP=%.15f, timeArch=%.15f, clockOffset = %lld\n", frame->timeStamp, getMonotonicTime(), getRealTime(), getTimeEpoch(), getDspClock(), getArchClock(), clockOffset );
#endif

   countP++;
}

void EagleCamera::onVideoFrame( ICameraFrame *frame )
{
   static uint32_t countV = 0;

   if( eagleCaptureParams.outputFormat == BlurCameraParams::RAW_FORMAT )
   {
      convertAtlRawToGrayScale( frame, eagleCaptureParams.pSize[0], eagleCaptureParams.pSize[1] );
   }
   callback( frame->timeStamp + clockOffset, frame->data );

#ifdef PRINT_CLOCKS
   printf( "Cam Frame timestamp=%lld, monotonic=%lld, realtime=%lld, timeEpoch=%lld, timeDSP=%.15f, timeArch=%.15f, clockOffset = %lld\n", frame->timeStamp, getMonotonicTime(), getRealTime(), getTimeEpoch(), getDspClock(), getArchClock(), clockOffset );
#endif
   countV++;
}

void EagleCamera::setCaptureParams( BlurCameraParams& params )
{
   eagleCaptureParams = params;
}

int EagleCamera::initialize( int camId )
{
   int rc;
   rc = ICameraDevice::createInstance( camId, &camera_ );
   if( rc != 0 )
   {
      printf( "could not open camera %d\n", camId );
      return rc;
   }
   camera_->addListener( this );

   rc = atlParams.init( camera_ );
   if( rc != 0 )
   {
      printf( "failed to init parameters\n" );
      ICameraDevice::deleteInstance( &camera_ );
   }

   return rc;
}

int EagleCamera::setParameters()
{
   /* temp: using hard-coded values to test the api
      need to add a user interface or script to get the values to test*/

      int wbModeIdx = 2;
      int isoModeIdx = 5;   /// iso800
   int pFpsIdx = -1;
   int vFpsIdx = -1;

#if defined (ISA_8x74_v7a)

   int defaultPFps = 3; /// 30 fps
   int defaultVFps = 3; /// 30 fps

   int pSizeIdx = 2;   // 640 , 480
   int vSizeIdx = 2;   // 640 , 480

#endif

   int rc;

   cameraCaps.pSizes = atlParams.getSupportedPreviewSizes();
   cameraCaps.vSizes = atlParams.getSupportedVideoSizes();
   cameraCaps.focusModes = atlParams.getSupportedFocusModes();
   cameraCaps.wbModes = atlParams.getSupportedWhiteBalance();
   cameraCaps.isoModes = atlParams.getSupportedISO();
   cameraCaps.brightness = atlParams.getSupportedBrightness();
   cameraCaps.sharpness = atlParams.getSupportedSharpness();
   cameraCaps.contrast = atlParams.getSupportedContrast();
   cameraCaps.previewFpsRanges = atlParams.getSupportedPreviewFpsRanges();
   cameraCaps.videoFpsValues = atlParams.getSupportedVideoFps();

   cameraExposureHist[0] = eagleCaptureParams.exposure;
   cameraExposureHist[1] = eagleCaptureParams.exposure;

   ImageSize frameSize;
   frameSize.width = eagleCaptureParams.pSize[0];
   frameSize.height = eagleCaptureParams.pSize[1];

   if( eagleCaptureParams.captureMode == BlurCameraParams::PREVIEW )
   {
      printf( "settings preview size %dx%d\n", frameSize.width, frameSize.height );
      atlParams.setPreviewSize( frameSize );
   }
   else if( eagleCaptureParams.captureMode == BlurCameraParams::VIDEO )
   {
      printf( "settings video size %dx%d\n", frameSize.width, frameSize.height );
      atlParams.setVideoSize( frameSize );
   }

   if( eagleCaptureParams.func == BlurCameraParams::CAM_FUNC_HIRES )
   {
      printf( "setting ISO mode: %s\n", cameraCaps.isoModes[isoModeIdx].c_str() );
      atlParams.setISO( cameraCaps.isoModes[isoModeIdx] );

      // Since not all focus modes are supported, we need to ensure that 
      // the user requested focus mode is a valid one
      printf
      (
         "Calculating focus mode. Available: %d Specified: %d \n",
         cameraCaps.focusModes.size(),
         (int)eagleCaptureParams.focusMode
      );
      if( cameraCaps.focusModes.size() > (int)eagleCaptureParams.focusMode )
      {
         printf( "setting focus mode: %s\n", cameraCaps.focusModes[eagleCaptureParams.focusMode].c_str() );
         atlParams.setFocusMode( cameraCaps.focusModes[eagleCaptureParams.focusMode] );
      }
      else
      {
         // Set the default focus mode
         if( cameraCaps.focusModes.size() == 0 )
         {
            printf( "ERROR: Cannot set the focus mode as it's not supported\n" );
         }
         else
         {
            printf
            (
               "Requested focus mode not available. Setting to default mode: %s\n",
               cameraCaps.focusModes[BlurCameraParams::FOCUS_AUTO].c_str()
            );
            atlParams.setFocusMode
            (
               cameraCaps.focusModes[BlurCameraParams::FOCUS_AUTO]
            );
         }
      }

      printf("setting WB mode: %s\n", cameraCaps.wbModes[wbModeIdx].c_str());
      atlParams.setWhiteBalance(cameraCaps.wbModes[wbModeIdx]);
   }

#if defined (ISA_8x74_v7a)

   if( cameraCaps.previewFpsRanges.size() <= defaultPFps )
   {
      printf( "default preview fps index %d greater than number of supported fps ranges %d \n setting to %d as default\n", defaultPFps, cameraCaps.previewFpsRanges.size(), cameraCaps.previewFpsRanges.size() - 1 );
      defaultPFps = cameraCaps.previewFpsRanges.size() - 1;
   }
   if( cameraCaps.videoFpsValues.size() <= defaultVFps )
   {
      printf( "default video fps index %d greater than number of supported fps ranges %d \n setting to %d as default\n", defaultVFps, cameraCaps.videoFpsValues.size(), cameraCaps.videoFpsValues.size() - 1 );
      defaultVFps = cameraCaps.videoFpsValues.size() - 1;
   }
   for( int i = 0; i < cameraCaps.previewFpsRanges.size(); ++i )
   {
      if( (cameraCaps.previewFpsRanges[i].max) / 1000 == eagleCaptureParams.frameRate )
      {
         pFpsIdx = i;
         break;
      }
   }
   for( int i = 0; i < cameraCaps.videoFpsValues.size(); ++i )
   {
      if( cameraCaps.videoFpsValues[i] == eagleCaptureParams.frameRate )
      {
         vFpsIdx = i;
         break;
      }
   }
   if( pFpsIdx == -1 )
   {
      printf( "couldnt find preview fps index for requested framerate %f setting default index %d \n", eagleCaptureParams.frameRate, defaultPFps );
      pFpsIdx = defaultPFps;
   }
   if( vFpsIdx == -1 )
   {
      printf( "couldnt find video fps index for requested framerate %f setting default index %d \n", eagleCaptureParams.frameRate, defaultVFps );
      vFpsIdx = defaultVFps;
   }

   printf( "setting preview fps range(idx %d): %d, %d\n", (int)pFpsIdx, (int)cameraCaps.previewFpsRanges[pFpsIdx].min,
           (int)cameraCaps.previewFpsRanges[pFpsIdx].max );
   atlParams.setPreviewFpsRange( cameraCaps.previewFpsRanges[pFpsIdx] );
   printf( "setting video fps(idx %d): %d\n", vFpsIdx, cameraCaps.videoFpsValues[vFpsIdx] );
   atlParams.setVideoFPS( cameraCaps.videoFpsValues[vFpsIdx] );

#else

   atlParams.set("preview-format", "bayer-rggb");
   atlParams.set("picture-format", "bayer-mipi-10gbrg");
   atlParams.set("raw-size", "640x480");

   // rc = setFPSindex( DEFAULT_CAMERA_FPS, pFpsIdx, vFpsIdx );

#endif

   if( eagleCaptureParams.outputFormat == BlurCameraParams::RAW_FORMAT )
   {
      printf( "setting outputFormat RAW_FORMAT\n" );
      atlParams.set( "preview-format", "bayer-rggb" );
      atlParams.set( "picture-format", "bayer-mipi-10gbrg" );
      atlParams.set( "raw-size", "640x480" );

#if not defined (ISA_8x74_v7a)
      atlParams.setPreviewFormat( FORMAT_RAW10 );
#endif

   }
   if( eagleCaptureParams.outputFormat == BlurCameraParams::NV12_FORMAT )
   {
      printf( "setting outputFormat nv12\n" );
      atlParams.set( "preview-format", "nv12" );
      atlParams.set( "raw-size", "640x480" );
   }
   printf( "set up params \n" );
   int ret = atlParams.commit();
   printf( "set up params done \n" );
   return ret;
}

bool EagleCamera::init()
{
   int n = getNumberOfCameras();

   printf( "num_cameras = %d\n", n );

   if( n < 1 )
   {
      printf( "No cameras found.\n" );
      return false;
   }

   camId = -1;

#if defined (ISA_8x74_v7a)

   /* find camera based on function */
   for( int i = 0; i < n; i++ )
   {
      CameraInfo info;
      getCameraInfo( i, info );
      printf( "Camera Id: %d , Function id: %d Requested: %d \n", i, info.func, eagleCaptureParams.func );
      if( info.func == eagleCaptureParams.func )
      {
         camId = i;
      }
   }

   if( camId == -1 )
   {
      printf( "Camera not found. \n" );
      exit( 1 );
   }


#else

   camId = eagleCaptureParams.func;

#endif

   printf( "initializing camera id=%d\n", camId );

   int ret = initialize( camId );
   if( ret != 0 )
   {
      printf( "ERR: initializing camera with %d id failed with err %d \n", camId, ret );
      return false;
   }

   ret = setParameters();
   if( ret != 0 )
   {
      printf( "ERR: initializing camera with %d id failed with err %d \n", camId, ret );
      return false;
   }

   return true;
}

bool EagleCamera::deinit()
{
   bool ok = false;
   /* release camera device */
   ICameraDevice::deleteInstance( &camera_ );
   return ok;
}

void EagleCamera::printCapabilities()
{
   printf( "Camera capabilities\n" );
   cameraCaps.pSizes = atlParams.getSupportedPreviewSizes();
   cameraCaps.vSizes = atlParams.getSupportedVideoSizes();
   cameraCaps.focusModes = atlParams.getSupportedFocusModes();
   cameraCaps.wbModes = atlParams.getSupportedWhiteBalance();
   cameraCaps.isoModes = atlParams.getSupportedISO();
   cameraCaps.brightness = atlParams.getSupportedBrightness();
   cameraCaps.sharpness = atlParams.getSupportedSharpness();
   cameraCaps.contrast = atlParams.getSupportedContrast();
   cameraCaps.previewFpsRanges = atlParams.getSupportedPreviewFpsRanges();
   cameraCaps.videoFpsValues = atlParams.getSupportedVideoFps();

   printf( "available preview sizes:\n" );
   for( size_t i = 0; i < cameraCaps.pSizes.size(); i++ )
   {
      printf( "%d: %d x %d\n", i, cameraCaps.pSizes[i].width, cameraCaps.pSizes[i].height );
   }
   printf( "available video sizes:\n" );
   for( size_t i = 0; i < cameraCaps.vSizes.size(); i++ )
   {
      printf( "%d: %d x %d\n", i, cameraCaps.vSizes[i].width, cameraCaps.vSizes[i].height );
   }
   printf( "available focus modes:\n" );
   for( size_t i = 0; i < cameraCaps.focusModes.size(); i++ )
   {
      printf( "%d: %s\n", i, cameraCaps.focusModes[i].c_str() );
   }
   printf( "available white balance modes:\n" );
   for( size_t i = 0; i < cameraCaps.wbModes.size(); i++ )
   {
      printf( "%d: %s\n", i, cameraCaps.wbModes[i].c_str() );
   }
   printf( "available ISO modes:\n" );
   for( size_t i = 0; i < cameraCaps.isoModes.size(); i++ )
   {
      printf( "%d: %s\n", i, cameraCaps.isoModes[i].c_str() );
   }
   printf( "available brightness values:\n" );
   printf( "min=%d, max=%d, step=%d\n", cameraCaps.brightness.min,
           cameraCaps.brightness.max, cameraCaps.brightness.step );
   printf( "available sharpness values:\n" );
   printf( "min=%d, max=%d, step=%d\n", cameraCaps.sharpness.min,
           cameraCaps.sharpness.max, cameraCaps.sharpness.step );
   printf( "available contrast values:\n" );
   printf( "min=%d, max=%d, step=%d\n", cameraCaps.contrast.min,
           cameraCaps.contrast.max, cameraCaps.contrast.step );

   printf( "available preview fps ranges:\n" );
   for( size_t i = 0; i < cameraCaps.previewFpsRanges.size(); i++ )
   {
      printf( "%d: [%d, %d]\n", i, cameraCaps.previewFpsRanges[i].min,
              cameraCaps.previewFpsRanges[i].max );
   }
   printf( "available video fps values:\n" );
   for( size_t i = 0; i < cameraCaps.videoFpsValues.size(); i++ )
   {
      printf( "%d: %d\n", i, (int)cameraCaps.videoFpsValues[i] );
   }
}

bool EagleCamera::start()
{
   int ret = 0;
   if( eagleCaptureParams.captureMode == BlurCameraParams::PREVIEW )
   {
      printf( "start preview\n" );
      ret = camera_->startPreview();
      if( 0 != ret )
      {
         printf( "ERR: start preview failed %d\n", ret );
      }
   }
   else if( eagleCaptureParams.captureMode == BlurCameraParams::VIDEO )
   {
      printf( "start recording\n" );
      ret = camera_->startRecording();
      if( 0 != ret )
      {
         printf( "ERR: start recording failed %d\n", ret );
      }
   }

   if( 0 != ret )
      return false;

   //Copy values, as setExposureAndGain only updates if different values
   //DK: This is a hack, but should work and not hurt
   float tmpExposure = eagleCaptureParams.exposure;
   float tmpGain = eagleCaptureParams.gain;

   eagleCaptureParams.exposure = 0.f;
   eagleCaptureParams.gain = 0.f;

   setExposureAndGain( tmpExposure, tmpGain );

   return true;
}

void EagleCamera::setExposureAndGain( float exposure, float gain )
{
   if( exposure == eagleCaptureParams.exposure && gain == eagleCaptureParams.gain )
   {
      //parameters are the same, don't have to set again
      return;
   }

   //printf("setExposureAndGain: %f %f \n", exposure, gain);

   if( exposure >= 0.f && exposure <= 1.f /*&& eagleCaptureParams.func == BlurCameraParams::CAM_FUNC_OPTIC_FLOW*/ )
   {
      eagleCaptureParams.exposure = exposure;
      cameraExposureHist[1] = cameraExposureHist[0];
      cameraExposureHist[0] = exposure;
      int exposureValue = MIN_EXPOSURE_VALUE + eagleCaptureParams.exposure * (MAX_EXPOSURE_VALUE - MIN_EXPOSURE_VALUE);
#if defined (ISA_8x74_v7a)
      char buffer[33];
      snprintf( buffer, sizeof( buffer ), "%d", exposureValue );
      atlParams.set( "qc-exposure-manual", buffer );
#else
      atlParams.setManualExposure( exposureValue );
#endif
      printf("Setting exposure value = %d \n", exposureValue);
   }
   if( gain >= 0.f && gain <= 1.f /*&& eagleCaptureParams.func == BlurCameraParams::CAM_FUNC_OPTIC_FLOW*/ )
   {
      eagleCaptureParams.gain = gain;
      int gainValue = MIN_GAIN_VALUE + eagleCaptureParams.gain * (MAX_GAIN_VALUE - MIN_GAIN_VALUE);
#if defined (ISA_8x74_v7a)
      char buffer[33];
      snprintf( buffer, sizeof( buffer ), "%d", gainValue );
      atlParams.set( "qc-gain-manual", buffer );
#else
      atlParams.setManualGain( gainValue );
#endif
      printf("Setting gain value = %d \n", gainValue);
   }
   atlParams.commit();
}

bool EagleCamera::stop()
{
   if( eagleCaptureParams.captureMode == BlurCameraParams::PREVIEW )
   {
      printf( "stop preview\n" );
      camera_->stopPreview();
      printf( "stop preview done\n" );
   }
   else if( eagleCaptureParams.captureMode == BlurCameraParams::VIDEO )
   {
      printf( "stop recording\n" );
      camera_->stopRecording();
   }
   return true;
}

void EagleCamera::addCallback( CameraCallback _callback )
{
   callback = _callback;
}

#if defined( ISA_8x96_v7a)

/* FUNCTION: setFPSindex
*
* scans through the supported fps values and returns index of
* requested fps in the array of supported fps
*
* @param fps      : Required FPS  (Input)
* @param pFpsIdx  : preview fps index (output)
* @param vFpsIdx  : video fps index   (output)
*
*/  
int setFPSindex( int fps, int &pFpsIdx, int &vFpsIdx )
{

   int defaultPrevFPSIndex = -1;
   int defaultVideoFPSIndex = -1;
   size_t i;
   int rc = 0;
   for( i = 0; i < cameraCaps.previewFpsRanges.size(); i++ )
   {
      if( (cameraCaps.previewFpsRanges[i].max) / 1000 == fps )
      {
         pFpsIdx = i;
         break;
      }
      if( (cameraCaps.previewFpsRanges[i].max) / 1000 == DEFAULT_CAMERA_FPS )
      {
         defaultPrevFPSIndex = i;
      }
   }
   if( i >= cameraCaps.previewFpsRanges.size() )
   {
      if( defaultPrevFPSIndex != -1 )
      {
         pFpsIdx = defaultPrevFPSIndex;
      }
      else
      {
         pFpsIdx = -1;
         rc = -1;
      }
   }

   for( i = 0; i < cameraCaps.videoFpsValues.size(); i++ )
   {
      if( fps == 30 * cameraCaps.videoFpsValues[i] )
      {
         vFpsIdx = i;
         break;
      }
      if( DEFAULT_CAMERA_FPS == 30 * cameraCaps.videoFpsValues[i] )
      {
         defaultVideoFPSIndex = i;
      }
   }
   if( i >= cameraCaps.videoFpsValues.size() )
   {
      if( defaultVideoFPSIndex != -1 )
      {
         vFpsIdx = defaultVideoFPSIndex;
      }
      else
      {
         vFpsIdx = -1;
         rc = -1;
      }
   }
   return rc;
}

#endif
