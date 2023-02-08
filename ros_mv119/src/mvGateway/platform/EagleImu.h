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

#ifndef _EAGLE_IMU_H_
#define _EAGLE_IMU_H_

#include <pthread.h>
/// for using direct imu callbacks
#include <sensor-imu/sensor_datatypes.h>
#include <sensor-imu/sensor_imu_api.h>
#include <cstdint> 
#include <time.h>
class EagleImu
{
public:
    typedef void( *ImuCallback )(float, float, float, int64_t);
    EagleImu();
    ~EagleImu();
    
    bool init();

    void stopCapturing();
    
    bool deinit();
    bool start();

    void addAccelCallback(ImuCallback callback);
    void addGyroCallback(ImuCallback callback);
    
    void invokeAccelCallback( float valX, float valY, float valZ, int64_t timeStamp)
    {
        if( !accelCallback )
        {
            return;
        }
        accelCallback( valX, valY, valZ, timeStamp );
    }
    void invokeGyroCallback( float valX, float valY, float valZ, int64_t timeStamp )
    {
        if( !gyroCallback )
        {
            return;
        }
        gyroCallback( valX, valY, valZ, timeStamp );
    }

    int16_t getData( sensor_imu* dataArray, int32_t max_count, int32_t* available_imu_data )
    {
        if( !sensorHandlePtr )
        {
            return -1;
        }
        return sensor_imu_attitude_api_get_imu_raw( sensorHandlePtr, dataArray, max_count, available_imu_data );
    }

    void setRunning( bool val ) { running = val; }
    
    bool isRunning() { return running; }

protected:

    bool running;
    
    ImuCallback gyroCallback, accelCallback;
    void* gyroUserData;
    void* accelUserData;

    sensor_handle* sensorHandlePtr;
    pthread_t imuPollThread;
};

#endif
