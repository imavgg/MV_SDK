/***************************************************************************//**
@internal
   Copyright 2015 Qualcomm Technologies, Inc.  All rights reserved.
   Confidential & Proprietary.
*******************************************************************************/

#include "SequenceWriterModule.h"



SequenceWriterModule::SequenceWriterModule()
{
    mMVSRWPtr = NULL;
    mCameraManagerPtr = NULL;
    mImuManagerPtr = NULL;
    mRunning = false;
    mInitialized = false;
    mPlatformAttitudeArray = new sensor_attitude[ImuManager::MAX_BUF_SIZE];
    mMvAttitudeArray = new mvAttitudeData[ImuManager::MAX_BUF_SIZE];
}



SequenceWriterModule::~SequenceWriterModule()
{
    if( mPlatformAttitudeArray != nullptr )
       delete [] mPlatformAttitudeArray;

    if( mMvAttitudeArray != nullptr )
       delete [] mMvAttitudeArray;
}



bool SequenceWriterModule::init( ImuManager* imuManager, CameraManager* cameraManager, SR_Configuration* srConfig )
{
    if( imuManager == NULL || cameraManager == NULL )
        return false;

    if( !mInitialized )
    {
        if( srConfig )
            mSwConfig = *srConfig;
        mImuManagerPtr = imuManager;
        mCameraManagerPtr = cameraManager;
        mImuManagerPtr->addNewListener( this );
        mCameraManagerPtr->addNewListener( this );

        mvMonoCameraInit cameraInit;
        cameraInit.name = "monoCamera";
        cameraInit.width = mCameraManagerPtr->getCameraConfig().pSize[0];
        cameraInit.height = mCameraManagerPtr->getCameraConfig().pSize[1];


        //Initialize Stereo as well, have to find a better way
        mvStereoCameraInit stereoInit;
        stereoInit.name = "stereoCamera";
        stereoInit.width = mCameraManagerPtr->getCameraConfig().pSize[0];
        stereoInit.height = mCameraManagerPtr->getCameraConfig().pSize[1];


        mMVSRWPtr = mvSRW_Writer_Initialize( mSwConfig.recordDirectory.c_str(), &cameraInit, &stereoInit );
        if( mMVSRWPtr == NULL )
        {
            std::cout << "MV, ERROR, failed initializing mvSRW" << std::endl;
            return false;
        }

        mCameraManagerPtr->enableCPALogging( mMVSRWPtr );

        mInitialized = true;
    }
    return true;
}



bool SequenceWriterModule::deinit()
{
   if( mSwThread.joinable() )
      mSwThread.join();

   mvSRW_Writer_Deinitialize( mMVSRWPtr );

   return true;
}



bool SequenceWriterModule::start()
{
    mRunning = true;
    std::cout << "MV, INFO, creating mvSRW thread " << std::endl;
    mSwThread = std::thread( &SequenceWriterModule::sequenceWriterProcessingMain, this );
    return true;
}



bool SequenceWriterModule::stop()
{
    mRunning = false;
    std::cout << "MV, INFO, stopped mvSRW thread " << std::endl;
    return true;
}



void SequenceWriterModule::sequenceWriterProcessingMain()
{
    if( mMVSRWPtr == NULL || mCameraManagerPtr == NULL || mImuManagerPtr == NULL
        || mInitialized == false )
        return;

    int64_t curTimeStamp;
    size_t nBytes = mCameraManagerPtr->getCameraConfig().pSize[0] * mCameraManagerPtr->getCameraConfig().pSize[1];
    unsigned char * curFrameBufferL = new unsigned char[nBytes];
    unsigned char * curFrameBufferR = new unsigned char[nBytes];
    int64_t curFrameId = 0, minFrameId=-1;
    while( mRunning )
    {
        if( mSwConfig.isStereo )
        {
            curFrameId = mCameraManagerPtr->getNextFrame( minFrameId, &curTimeStamp, curFrameBufferL, curFrameBufferR );
            if ( curFrameId >= 0 )
               mvSRW_Writer_AddStereoImage( mMVSRWPtr, curTimeStamp, curFrameBufferL, curFrameBufferR );
        }
        else
        {
            curFrameId = mCameraManagerPtr->getNextFrame( minFrameId, &curTimeStamp, curFrameBufferL );
            if ( curFrameId >= 0 )
               mvSRW_Writer_AddImage( mMVSRWPtr, curTimeStamp, curFrameBufferL );
        }
        minFrameId = curFrameId;
    }

    delete[] curFrameBufferL;
    delete[] curFrameBufferR;

}



void SequenceWriterModule::newAccelDataReceived( float valX, float valY, float valZ, int64_t timestamp )
{
    mvSRW_Writer_AddAccel( mMVSRWPtr, timestamp, valX, valY, valZ );
}

void SequenceWriterModule::newGyroDataReceived( float valX, float valY, float valZ, int64_t timestamp )
{
    mvSRW_Writer_AddGyro( mMVSRWPtr, timestamp, valX, valY, valZ );
    if( mSwConfig.captureAttitude )
    {
        int nAttitudes = 0;
        mImuManagerPtr->getAttitudeBuffered( mPlatformAttitudeArray, ImuManager::MAX_BUF_SIZE, &nAttitudes );
        for( auto i = 0; i < nAttitudes; ++i )
        {
            mvAttitudeData* curDst = (mvAttitudeData*)mMvAttitudeArray + i;
            sensor_attitude* curSrc = (sensor_attitude*)mPlatformAttitudeArray + i;
            curDst->timestamp = curSrc->timestamp_in_us * 1000;
            memcpy( &curDst->rotation_matrix, &curSrc->rotation_matrix, mvAttitudeData::ATTITUDE_MAT_SIZE * sizeof( float32_t ) );
        }
        mvSRW_Writer_AddAttitude( mMVSRWPtr, mMvAttitudeArray, nAttitudes );
    }
}

