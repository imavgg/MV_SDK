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

#ifndef _CAMERA_MANAGER_HELPER_H_
#define _CAMERA_MANAGER_HELPER_H_

using namespace camera;
// #if defined(ISA_8x74_v7a)
#include "camera.h"
#include "camera_parameters.h"
// #else
// #include <libcamera/camera.h>
// #include <libcamera/camera_parameters.h>
// #endif

#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

class CameraManagerHelper
{
public:

   static void convertAtlRawToGrayScale( ICameraFrame *frame, int width, int height )
   {
      int destSize = width * height;
      unsigned char* imageBuf = frame->data;
      for( int i = 0; 4 * i < destSize; ++i )
      {
         memcpy( reinterpret_cast<uint8_t*>(imageBuf) + i * 4, reinterpret_cast<uint8_t*>(frame->data) + i * 5, 4 );
      }
   }

   static void convertAtlRawToGrayScale( unsigned char *srcFrame, unsigned char *dstFrame, int width, int height )
   {
      int destSize = width * height;
      for( int i = 0; 4 * i < destSize; ++i )
      {
         memcpy( reinterpret_cast<uint8_t*>(dstFrame) + i * 4, reinterpret_cast<uint8_t*>(srcFrame) + i * 5, 4 );
      }
   }

   static void convertAtlRawToGrayScale16( ICameraFrame *frame, int width, int height )
   {
      int nPixels = width * height;


      unsigned char* imageBuf = frame->data;
      uint16_t pL = 0, p0H = 0, p1H = 0, p2H = 0, p3H = 0, mask = 0x0003;
      for( int i = 0; 4 * i < nPixels; i++ )
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

   static void convertAtlRawToGrayScale16( unsigned char *srcFrame, unsigned char *dstFrame, int width, int height )
   {
      int nPixels = width * height;


      uint16_t pL = 0, p0H = 0, p1H = 0, p2H = 0, p3H = 0, mask = 0x0003;
      for( int i = 0; 4 * i < nPixels; i++ )
      {
         p0H = *(reinterpret_cast<uint8_t*>(srcFrame) + i * 5 + 0);
         p1H = *(reinterpret_cast<uint8_t*>(srcFrame) + i * 5 + 1);
         p2H = *(reinterpret_cast<uint8_t*>(srcFrame) + i * 5 + 2);
         p3H = *(reinterpret_cast<uint8_t*>(srcFrame) + i * 5 + 3);
         pL = *(reinterpret_cast<uint8_t*>(srcFrame) + i * 5 + 4);
         *(reinterpret_cast<uint16_t*>(dstFrame) + i * 4 + 0) = (uint16_t)((p0H << 2) | ((pL >> 6) & mask));
         *(reinterpret_cast<uint16_t*>(dstFrame) + i * 4 + 1) = (uint16_t)((p1H << 2) | ((pL >> 4) & mask));
         *(reinterpret_cast<uint16_t*>(dstFrame) + i * 4 + 2) = (uint16_t)((p2H << 2) | ((pL >> 2) & mask));
         *(reinterpret_cast<uint16_t*>(dstFrame) + i * 4 + 3) = (uint16_t)((p3H << 2) | ((pL >> 0) & mask));

      }
   }

   static int getImageSizeInBytes( int width, int height )
   {
      return width*height;
   }

   static int64_t getMonotonicTime( void )
   {
      struct timespec t;
      clock_gettime( CLOCK_MONOTONIC, &t );
      uint64_t timeNanoSecMonotonic = t.tv_sec * 1000000000ULL + t.tv_nsec;
      return (int64_t)timeNanoSecMonotonic;
   }

   static int64_t getRealTime( void )
   {
      struct timespec t;
      clock_gettime( CLOCK_REALTIME, &t );
      uint64_t timeNanoSecRealTime = t.tv_sec * 1000000000ULL + t.tv_nsec;
      return (int64_t)timeNanoSecRealTime;
   }

   static int64_t getTimeEpoch( void )
   {
      time_t now = time( 0 );
      uint64_t timeNanoSecEpoch = now * 1000000000ULL;
      return (int64_t)timeNanoSecEpoch;
   }

   static double getDspClock( void )
   {
      static const char qdspTimerTickPath[] = "/sys/kernel/boot_adsp/qdsp_qtimer";
      static const double clockFreq = 1 / 19.2;

      char qdspTicksStr[20] = "";
      FILE* qdspClockfp = fopen( qdspTimerTickPath, "r" );
      if( qdspClockfp == nullptr )
      {
         std::cerr << "MV, CameraMangerHelper, ERROR: could not open" << qdspTimerTickPath << std::endl;
         return -1;
      }
      fread( qdspTicksStr, 16, 1, qdspClockfp );
      int64_t qdspTicks = strtoll( qdspTicksStr, 0, 16 );
      fclose( qdspClockfp );

      return qdspTicks*clockFreq*1e3;
   }

   static double getArchClock( void )
   {
      const char archTimerTickPath[] = "/sys/kernel/boot_adsp/arch_qtimer";
      char archTicksStr[20] = "";
      static const double clockFreq = 1 / 19.2;
      FILE* archClockfp = fopen( archTimerTickPath, "r" );
      if( archClockfp == nullptr )
      {
         std::cout << "MV, CameraMangerHelper, ERROR: could not open" << archTimerTickPath << std::endl;
         return -1;
      }
      fread( archTicksStr, 16, 1, archClockfp );
      int64_t archTicks = strtoll( archTicksStr, 0, 16 );
      fclose( archClockfp );

      return archTicks*clockFreq*1e3;
   }

   ///Currently we assume camera uses monotonic clock
   static int64_t findClocksOffsetForCamera()
   {
      int64_t dspClock = (int64_t)getDspClock();
      int64_t monotonicClock = getMonotonicTime();
      int64_t clockOffset = dspClock - monotonicClock;
      return clockOffset;
   }

};

#endif
