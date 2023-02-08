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

#ifndef _CAMERA_MANAGER_CONFIG_H_
#define _CAMERA_MANAGER_CONFIG_H_


#include "camera.h"
#include "camera_parameters.h"
#include "mvCPA.h"

using namespace camera;


class CameraCapabilities
{
public:
    std::vector<ImageSize> pSizes, vSizes;
    std::vector<std::string> focusModes, wbModes, isoModes;
    Range brightness, sharpness, contrast;
    std::vector<Range> previewFpsRanges;
    std::vector<VideoFPS> videoFpsValues;
};



class CameraManagerConfig
{
public:

    enum OutputFormatType
    {
        YUV_FORMAT,
        RAW_FORMAT,
        NV12_FORMAT,
    };


    enum CameraMangerOutputFormatType
    {
        IMAGE_8_BIT_GRAY,
        IMAGE_16_BIT_GRAY,
    };


#ifdef _HAL3_CAMERA_
    enum CamFunction {
        CAM_FUNC_HIRES = 0,
        CAM_FUNC_OPTIC_FLOW = 1,
        CAM_FUNC_RIGHT_SENSOR = 4,
        CAM_FUNC_STEREO = 2,
        CAM_FUNC_LEFT_SENSOR = 3,
        CAM_FUNC_STEREO_RAW = 5,
        CAM_FUNC_MAX,
    };
#else
    enum CamFunction
    {
       CAM_FUNC_HIRES = 0,
       CAM_FUNC_OPTIC_FLOW = 1,
       CAM_FUNC_RIGHT_SENSOR = 2,
       CAM_FUNC_STEREO = 3,
       CAM_FUNC_LEFT_SENSOR = 4,
       CAM_FUNC_STEREO_RAW = 5
    };
#endif


    enum CaptureMode
    {
        PREVIEW,
        VIDEO
    };


    enum CaptureResolution
    {
        QVGA_320_240 = 0,
        VGA_640_480 = 1,
        HD_1280_720 = 2,
        FHD_1920_1080 = 3,
        UHD_3840_2160 = 4,
        XGA_1024_768 = 5
    };


    CameraManagerConfig()
    {
        // default values
        gain = 1.f;
        exposure = 1.0f;
        afmode = 4;
        cameraid = 0;
        frameRate = 30.0f;
        useCPA = false;
        cpaUpdateRate = 4; /// In tests we notice that there is a delay of 3 frames for the camera parameters to get applied.Hence
                           /// we wait for 3 frames before computing and applying new cpa values.
        captureMode = PREVIEW;
        outputFormat = YUV_FORMAT;
#ifndef _HAL3_CAMERA_
        minExposureValue = 0;
        // This value comes (FrameLength - 20)
        // FrameLength is set at 1716 for 30fps camera
        // Row period = 19.333 us
        // Exposure time = RowPeriod * ExposureValue
        maxExposureValue = 1696;
        minGainValue = 0;
        maxGainValue = 255;
        rowPeriodUs = 19.3333f;
#else
        minExposureValue = 53591;
        maxExposureValue = 16576864;
        minGainValue = 100;
        maxGainValue = 800;
        // Row period = 1 ns
        rowPeriodUs = 0.001f;
#endif
        maxFrameBufferSize = 4;
        resolution = VGA_640_480;
        pSize.resize( 2 );
        pSize[0] = 640;
        pSize[1] = 480;
        pStride = 640;
    }

    float gain;
    float exposure;
    int afmode;
    int cameraid;
    float frameRate;
    std::vector<int> pSize;
    int pStride;
    CaptureMode captureMode;
    CamFunction func;
    OutputFormatType outputFormat;
    CaptureResolution resolution;

    bool useCPA;
    int cpaUpdateRate;
    mvCPA_Configuration cpaConfig;
    float rowPeriodUs;

    int minExposureValue;
    int maxExposureValue;
    int minGainValue;
    int maxGainValue;
    size_t maxFrameBufferSize;
};

#endif
