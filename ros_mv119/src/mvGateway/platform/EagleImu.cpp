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

#include "EagleImu.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

#define BUF_SIZE 100

void *imuPollFunc( void *context )
{
    EagleImu *myImuPtr = (EagleImu *)context;

    if( !myImuPtr )
    {
        printf( "here with null context returning!\n" );
        pthread_exit( NULL );
    }

    int numSamples = 0;
    float accVal[3], gyrVal[3];
    myImuPtr->setRunning(true);
    float delta = 0.f;
    const float NORM_G = 9.80665f;

    sensor_imu * sensorDataPtr = new sensor_imu[BUF_SIZE];
    int64_t lastTimeStamp = 0;
    while( myImuPtr->isRunning() )
    {
        myImuPtr->getData( sensorDataPtr, BUF_SIZE, &numSamples );
        for( int j = 0; j < numSamples; ++j )
        {
            sensor_imu * curData = (sensor_imu *)sensorDataPtr + j;
            int64_t curTimeStampNs = (int64_t)curData->timestamp_in_us * 1000;
            // printf( "time %" PRIu64 "\n", curData->timestamp_in_us );
            if( lastTimeStamp != 0 )
            {
                delta = (curTimeStampNs - lastTimeStamp)*1e-6;
                if( delta > 50.0 )
                    printf( "SensorInterarrival > 50ms :%fms \n", delta );
            }
            lastTimeStamp = curTimeStampNs;
            //Assume that we have both accel and gyro every sample.
            accVal[0] = curData->linear_acceleration[0] * NORM_G;
            accVal[1] = curData->linear_acceleration[1] * NORM_G;
            accVal[2] = curData->linear_acceleration[2] * NORM_G;
            gyrVal[0] = curData->angular_velocity[0];
            gyrVal[1] = curData->angular_velocity[1];
            gyrVal[2] = curData->angular_velocity[2];
            myImuPtr->invokeGyroCallback( gyrVal[0], gyrVal[1], gyrVal[2], curTimeStampNs );
            myImuPtr->invokeAccelCallback( accVal[0], accVal[1], accVal[2], curTimeStampNs );
        }
    }
    delete[]( sensor_imu * )sensorDataPtr;
    /* Last thing that main() should do */
    pthread_exit( NULL );
}

EagleImu::EagleImu() 
{
    gyroCallback = NULL;
    accelCallback = NULL;
    running = false;
    sensorHandlePtr = NULL;
}


EagleImu::~EagleImu()
{
}

bool EagleImu::init()
{
    int retVal = 0;
    printf( "initializing sensor imu\n" );
    sensorHandlePtr = sensor_imu_attitude_api_get_instance();
    if( sensorHandlePtr == NULL )
    {
        printf("Error getting imu instance \n");
        return false;
    }
    printf( "Imu version %s \n", sensor_imu_attitude_api_get_version( sensorHandlePtr ) );

    printf("Initializing sensor imu \n");
    retVal = sensor_imu_attitude_api_initialize( sensorHandlePtr, SENSOR_CLOCK_SYNC_TYPE_MONOTONIC );
    if( retVal != 0 )
    {
        printf( "Error: Failed calling IMU initialize... rc[%d]\n", retVal );
        return false;
    }
    retVal = sensor_imu_attitude_api_wait_on_driver_init( sensorHandlePtr );
    return retVal != 0 ? false : true;
}

bool EagleImu::start()
{
    if( !sensorHandlePtr )
    {
        printf( "imu instance not initialized \n" );
        return false;
    }
    sensor_mpu_driver_settings imuSettings;
    sensor_imu_attitude_api_get_mpu_driver_settings( sensorHandlePtr, &imuSettings );
    if( imuSettings.is_initialized != 1 )
    {
        printf( "imu driver not initialized imuSettings.is_initialized=%d \n", imuSettings.is_initialized );
        return false;
    }

    printf("imu settings: \n");
    printf( "sample_rate_in_hz:  %d\n", imuSettings.sample_rate_in_hz );
    printf( "compass_sample_rate_in_hz:  %d\n", imuSettings.compass_sample_rate_in_hz );
    printf( "accel_lpf_in_hz:  %d\n", imuSettings.accel_lpf_in_hz );
    printf( "gyro_lpf_in_hz:  %d\n", imuSettings.gyro_lpf_in_hz );

    running = true;
    printf( "creating thread \n" );
    int rc = pthread_create( &imuPollThread, NULL, imuPollFunc, (void *)this );
    if( rc )
    {
        printf( "ERROR; return code from pthread_create() is %d\n", rc );
        if( sensorHandlePtr )
            sensor_imu_attitude_api_terminate( sensorHandlePtr );
        return false;
    }
    return true;
}

void EagleImu::stopCapturing()
{
    running = false;
}

bool EagleImu::deinit()
{
    // Adding to sync stoping of sensors
    int retVal;
    if( sensorHandlePtr )
        retVal = sensor_imu_attitude_api_terminate( sensorHandlePtr );
    return retVal == 0;
}

void EagleImu::addAccelCallback(ImuCallback _callback)
{
    accelCallback = _callback;
}

void EagleImu::addGyroCallback(ImuCallback _callback)
{
    gyroCallback = _callback;
}
