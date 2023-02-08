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

#include "ImuManager.h"


void ImuManager::ThreadMain()
{
    int numSamples = 0;
    float delta = 0.f;
    sensor_imu* sensorDataPtr = new sensor_imu[MAX_BUF_SIZE];
    int64_t lastTimeStamp = 0;

    if( !mSensorHandlePtr )
    {
       delete [] sensorDataPtr;
       std::cout << "MV, ERROR, In the IMU manager thread without initializing." << std::endl;
       return;
    }

    while( mRunning )
    {
        sensor_imu_attitude_api_get_imu_raw( mSensorHandlePtr, sensorDataPtr, MAX_BUF_SIZE, &numSamples );
        for( int j = 0; j < numSamples; ++j )
        {
            sensor_imu* curData = (sensor_imu *)sensorDataPtr + j;
            int64_t curTimeStampNs = (int64_t)curData->timestamp_in_us * 1000;
            if( lastTimeStamp != 0 )
            {
                delta = (curTimeStampNs - lastTimeStamp)*1e-6;
                if( delta > 50.0 )
                    std::cout << "MV, WARN,SensorInterarrival > 50ms :"<< delta <<"ms"<< std::endl;
            }
            lastTimeStamp = curTimeStampNs;
            //Assume that we have both accel and gyro every sample.
            for( auto listener : mListeners )
            {
                listener->newGyroDataReceived( curData->angular_velocity[0], curData->angular_velocity[1], curData->angular_velocity[2], curTimeStampNs );
                listener->newAccelDataReceived( curData->linear_acceleration[0] * NORM_G, curData->linear_acceleration[1] * NORM_G, curData->linear_acceleration[2] * NORM_G, curTimeStampNs );
            }

        }
    }

    delete [] (sensor_imu*)sensorDataPtr;
}



ImuManager::ImuManager()
{
    mRunning = false;
    mSensorHandlePtr = NULL;
    mFlightStackEnabled = false;
}



ImuManager::~ImuManager()
{
    mSensorHandlePtr = NULL;
}



bool ImuManager::init()
{
    int retVal = 0;
    std::cout << "MV, INFO, getting sensor imu instance" <<std::endl;
    mSensorHandlePtr = sensor_imu_attitude_api_get_instance();
    if( mSensorHandlePtr == NULL )
    {
        std::cout << "MV, ERROR, Error getting imu instance "<< std::endl;
        return false;
    }
    std::cout << "Imu version " << sensor_imu_attitude_api_get_version( mSensorHandlePtr ) << std::endl;

    std::cout << "MV, INFO, initializing sensor imu" << std::endl;
    retVal = sensor_imu_attitude_api_initialize( mSensorHandlePtr, SENSOR_CLOCK_SYNC_TYPE_MONOTONIC );
    if( retVal != 0 )
    {
        std::cout << "MV, ERROR, Failed calling IMU initialize... rc[" << retVal <<"]" <<std::endl;
        return false;
    }

    retVal = sensor_imu_attitude_api_wait_on_driver_init( mSensorHandlePtr );
    if( retVal != 0 )
        std::cout << "MV, ERROR, Failed waiting to intialize imu... rc["<< retVal<<"]"<<std::endl;

    mFlightStackEnabled = sensor_imu_attitude_api_is_flight_stack_enabled( mSensorHandlePtr );

    return retVal != 0 ? false : true;
}



bool ImuManager::start()
{
    if( !mSensorHandlePtr )
    {
        std::cout << "MV, ERROR, imu instance not initialized " << std::endl;
        return false;
    }

    sensor_mpu_driver_settings imuSettings;
    sensor_imu_attitude_api_get_mpu_driver_settings( mSensorHandlePtr, &imuSettings );
    if( imuSettings.is_initialized != 1 )
    {
        std::cout << "MV, ERROR, imu driver not initialized imuSettings.is_initialized=" << imuSettings.is_initialized << std::endl;
        return false;
    }

    std::cout << "MV, INFO, imu settings: "<< std::endl;
    std::cout << "MV, INFO, sample_rate_in_hz:  " <<imuSettings.sample_rate_in_hz << std::endl;
    std::cout << "MV, INFO, compass_sample_rate_in_hz:  "<< imuSettings.compass_sample_rate_in_hz << std::endl;
    std::cout << "MV, INFO, accel_lpf_in_hz:  "<< imuSettings.accel_lpf_in_hz << std::endl;
    std::cout << "MV, INFO, gyro_lpf_in_hz:  "<< imuSettings.gyro_lpf_in_hz << std::endl;

    mRunning = true;
    std::cout << "MV, INFO, creating imu thread " << std::endl;
    mImuPollThread = std::thread( &ImuManager::ThreadMain, this );
    return true;
}



void ImuManager::stop()
{
    mRunning = false;
    if( mImuPollThread.joinable() ) mImuPollThread.join();
    std::cout << "MV, INFO, stopped imu thread " << std::endl;
}



bool ImuManager::deinit()
{
    if( mRunning )
        stop();

    if( mSensorHandlePtr )
        sensor_imu_attitude_api_terminate( mSensorHandlePtr );
    return true;
}



void ImuManager::addNewListener( ImuFrameListener* listener )
{
    mListeners.push_back( listener );
}



ImuFrameListener::~ImuFrameListener()
{};
