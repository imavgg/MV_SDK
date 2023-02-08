/***************************************************************************//**
@internal
   Copyright 2015 Qualcomm Technologies, Inc.  All rights reserved.
   Confidential & Proprietary.
*******************************************************************************/


#include "VISLAMModule.h"



VISLAMModule::VISLAMModule()
{
    mMVVislamPtr = NULL;
    mCameraManagerPtr = NULL;
    mImuManagerPtr = NULL;
    mRunning = false;
    mInitialized = false;
    memset( &mVislamPose, 0, sizeof( mvVISLAMPose ) );
    memset( &mVislamPointCloud, 0, MAX_VISLAM_TRACKING_PTS * sizeof( mvVISLAMMapPoint ) );
}



VISLAMModule::~VISLAMModule()
{
}



bool VISLAMModule::init( ImuManager* imuManager, CameraManager* cameraManager, mvVISLAMConfiguration* vislamConfig )
{
    if( imuManager == NULL || cameraManager == NULL )
        return false;

    if( !mInitialized )
    {
        if( vislamConfig )
            mVislamConfig = *vislamConfig;
        mImuManagerPtr = imuManager;
        mCameraManagerPtr = cameraManager;
        mImuManagerPtr->addNewListener( this );
        mCameraManagerPtr->addNewListener( this );

        mMVVislamPtr = mvVISLAM_Initialize( &mVislamConfig );

        if( mMVVislamPtr == NULL )
        {
            std::cout << "MV, ERROR, failed initializing mvVISLAM" << std::endl;
            return false;
        }
        mInitialized = true;
    }
    return true;
}



bool VISLAMModule::deinit()
{
    mvVISLAM_Deinitialize( mMVVislamPtr );
}



bool VISLAMModule::start()
{
    mRunning = true;
    std::cout << "MV, INFO, creating VISLAM thread " << std::endl;
    mVislamThread = std::thread( &VISLAMModule::vislamProcessingMain, this );
    return true;
}



bool VISLAMModule::stop()
{
    mRunning = false;
    if( mVislamThread.joinable() ) mVislamThread.join();
    std::cout << "MV, INFO, stopped VISLAM thread " << std::endl;
    return true;
}



void VISLAMModule::vislamProcessingMain()
{
    if( mMVVislamPtr == NULL || mCameraManagerPtr == NULL || mImuManagerPtr == NULL 
        || mInitialized == false )
        return;

    int64_t curTimeStamp;
    size_t nBytes = mCameraManagerPtr->getCameraConfig().pSize[0] * mCameraManagerPtr->getCameraConfig().pSize[1];
    unsigned char * curFrameBuffer = new unsigned char[nBytes];
    int64_t curFrameId = 0, minFrameId=-1;
    while( mRunning )
    {
        curFrameId = mCameraManagerPtr->getNextFrame( minFrameId, &curTimeStamp, curFrameBuffer );
        mvVISLAM_AddImage( mMVVislamPtr, curTimeStamp, curFrameBuffer, false );
        mVislamPose = mvVISLAM_GetPose( mMVVislamPtr );
        int nPoints = mvVISLAM_HasUpdatedPointCloud( mMVVislamPtr );
        if( nPoints > 0 )
            // Make sure gPointCloud has enough memory allocated to get nPoints back
            nPoints = mvVISLAM_GetPointCloud( mMVVislamPtr, mVislamPointCloud, MAX_VISLAM_TRACKING_PTS );
        minFrameId = curFrameId;
    }

    delete [] curFrameBuffer; 

}



void VISLAMModule::newAccelDataReceived( float valX, float valY, float valZ, int64_t timestamp )
{
    mvVISLAM_AddAccel( mMVVislamPtr, timestamp, valX, valY, valZ );
}



void VISLAMModule::newGyroDataReceived( float valX, float valY, float valZ, int64_t timestamp )
{
    mvVISLAM_AddGyro( mMVVislamPtr, timestamp, valX, valY, valZ );
}

