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

#ifndef _EAGLE_CAMERA_H_
#define _EAGLE_CAMERA_H_

#if defined (ISA_8x74_v7a)

    #include "/usr/include/camera_parameters.h"
    #include <vector>

    #include "/usr/include/camera.h"
    #define MIN_EXPOSURE_VALUE 0
     // This value comes (FrameLength - 20)
     // FrameLength is set at 1716 for 30fps camera
     // Row period = 19.333 us
     // Exposure time = RowPeriod * ExposureValue
    #define MAX_EXPOSURE_VALUE 1696

    #define MIN_GAIN_VALUE 0
    #define MAX_GAIN_VALUE 255

#else

    #include "/usr/include/camera.h"
    //#include "camera_log.h"
    #include "/usr/include/camera_parameters.h"
    #include <vector>
    #include "mvCPA.h"
    #include "mvSRW.h"

    #define DEFAULT_EXPOSURE_VALUE  3000000 //ns
    #define MIN_EXPOSURE_VALUE        19334 //ns
    #define MAX_EXPOSURE_VALUE     17942770 //ns
    #define DEFAULT_GAIN_VALUE  200      
    #define MIN_GAIN_VALUE 0
    #define MAX_GAIN_VALUE  1000
    #define QCAMERA_DUMP_LOCATION "/data/misc/camera/dumps/"

    #define DEFAULT_CAMERA_FPS 30
    #define MS_PER_SEC 1000
    #define NS_PER_MS 1000000
    #define NS_PER_US 1000
    #define MAX_BUF_SIZE 128

    using namespace std;
    using namespace camera;

#endif

// using namespace camera;

struct CameraCapabilities
{
    // can't find the type.
    std::vector<ImageSize> pSizes, vSizes;
    std::vector<std::string> focusModes, wbModes, isoModes;
    Range brightness, sharpness, contrast;
    std::vector<Range> previewFpsRanges;
    std::vector<VideoFPS> videoFpsValues;
};

struct BlurCameraParams
{
    enum OutputFormatType{
        YUV_FORMAT,
        RAW_FORMAT,
        NV12_FORMAT,
    };

    enum FocusModeType{
        FOCUS_AUTO = 0, // Also known as FIXED for newer boards which has only 1 mode
        FOCUS_INFINITY,
        FOCUS_MACRO,
        FOCUS_CONTINUOUS_VIDEO,
        FOCIUS_CONTINUS_PICTURE,
        FOCUS_MANUAL
    };

    enum CamFunction {
        CAM_FUNC_HIRES = 0,
        CAM_FUNC_OPTIC_FLOW = 1,
        CAM_FUNC_LEFT_SENSOR = 2,
        CAM_FUNC_STEREO = 3
    };

    enum CaptureMode
    {
        PREVIEW,
        VIDEO
    };
    BlurCameraParams()
    {
        // default values
        gain = 1.f;
        exposure = 1.0f;
        afmode = 4;
        cameraid = 0;
        frameRate = 30.0f;
        pSize.resize( 2 );
        pSize[0] = 1280;
        pSize[1] = 720;
        useIllControl = false;
        captureMode = PREVIEW;
        outputFormat = YUV_FORMAT;
        focusMode = FOCUS_AUTO;
    }
    float gain;
    float exposure;
    int afmode;
    int cameraid;
    float frameRate;
    bool useIllControl;
    std::vector<int> pSize;
    CaptureMode captureMode;
    CamFunction func;
    OutputFormatType outputFormat;
    FocusModeType focusMode;
};



class EagleCamera:  camera::ICameraListener
{
    public:

        typedef void( *CameraCallback )(int64_t, unsigned char *);

        EagleCamera();
        ~EagleCamera();

        void setCaptureParams(BlurCameraParams& params);
        // #if defined( ISA_8x96_v7a)
        //     int setFPSindex(int fps, int &pFpsIdx, int &vFpsIdx);
        // #endif

        void setExposureAndGain(float exposure, float gain);

        bool init();
        bool deinit();

        bool start();
        bool stop();

        void addCallback(CameraCallback callback);

        /* listener methods */

        virtual void onError();
        virtual void onPreviewFrame(ICameraFrame* frame);
        virtual void onVideoFrame(ICameraFrame* frame);
        void printCapabilities();
        float getCameraExposureTime();  // HAL1: in us

    private:
        void findClocksOffsetForCamera();
        int initialize(int camId);
        int setParameters();
        CameraCallback callback;

        BlurCameraParams eagleCaptureParams;
        CameraParams atlParams;
        CameraCapabilities cameraCaps;
        ICameraDevice* camera_;
        int camId;

        int64_t clockOffset;

        float cameraExposureHist[2];
        float exposureValueHAL3;
};

#endif
