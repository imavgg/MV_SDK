/***************************************************************************//**
@internal
   Copyright 2015 Qualcomm Technologies, Inc.  All rights reserved.
   Confidential & Proprietary.
*******************************************************************************/

#ifndef _VISLAM_MODULE_H_
#define _VISLAM_MODULE_H_

#include <thread>
#include <mvVISLAM.h>
#include "CameraManager.h"
#include "ImuManager.h"

class VISLAMModule : public CameraFrameListener, public ImuFrameListener
{
public:
    enum
    {
        MAX_VISLAM_TRACKING_PTS = 40
    };

    typedef void( *VISLAMCallBack )(int64_t, unsigned char *);

    VISLAMModule();
    virtual ~VISLAMModule();

    /* intializes the vislam module with appropriate config */
    bool init( ImuManager* imuManager, CameraManager* cameraManager, mvVISLAMConfiguration* vislamConfig );

    /* de intializes the vislam module */
    bool deinit();

    /* starts vislam processing */
    bool start();

    /* stops vislam processing */
    bool stop();

    //do nothing as vislam camera processing is slow. Dont want to do any processing in camera thread.
    void newCameraFrameReceived( int64_t timestamp, unsigned char * buffer ){};

    void newAccelDataReceived( float valX, float valY, float valZ, int64_t timestamp );
    void newGyroDataReceived( float valX, float valY, float valZ, int64_t timestamp );

private:
    void vislamProcessingMain();
    mvVISLAM* mMVVislamPtr;
    ImuManager* mImuManagerPtr;
    CameraManager* mCameraManagerPtr;

    mvVISLAMPose mVislamPose;
    mvVISLAMMapPoint mVislamPointCloud[MAX_VISLAM_TRACKING_PTS];
    mvVISLAMConfiguration mVislamConfig;
    bool mInitialized;
    bool mRunning;
    std::thread mVislamThread;

};
#endif
