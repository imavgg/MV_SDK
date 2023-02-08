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

#ifndef _IMU_MANAGER_H_
#define _IMU_MANAGER_H_

/// for using direct imu callbacks
#include <sensor-imu/sensor_datatypes.h>
#include <sensor-imu/sensor_imu_api.h>
#include <thread>
#include <vector>
#include <iostream>

class ImuFrameListener
{
public:
    virtual void newAccelDataReceived( float valX, float valY, float valZ, int64_t timestamp ) = 0;
    virtual void newGyroDataReceived( float valX, float valY, float valZ, int64_t timestamp ) = 0;
    virtual ~ImuFrameListener() = 0;
};

class ImuManager
{
public:
    ImuManager();
    ~ImuManager();
    enum
    {
        MAX_BUF_SIZE = 100
    };

    /* intializes the imu manager with appropriate config */
    bool init();  

    /* starts the imu data */
    bool start();

    /* stops the imu data */
    void stop();

    /* deinitializes the imu manager. */
    bool deinit();

    /**
    * Gets the imu data from the adsp.  This is a blocking call, if there
    * this no data.  On Adsp the numbers of samples bufferred is 100.  If the
    * buffer is full the oldest data is overwritten.
    * @param sensor_imu*
    *     A pointer to the sensor_imu array to get the imu from the adsp.
    * @param max_count
    *    The maximum size for the above array.
    * @param returned_imu_count
    *     the number of imu's samples returned.
    * @return
    *     0 = success
    *     other = failure;
    * @Note: The ImuData is in  the raw IMU Frame of the eagle board.
    */
    int16_t getData( sensor_imu* dataArray, int32_t max_count, int32_t* available_imu_data )
    {
        if( !mSensorHandlePtr || !dataArray )
        {
            return -1;
        }
        return sensor_imu_attitude_api_get_imu_raw( mSensorHandlePtr, dataArray, max_count, available_imu_data );
    }

    /**
    * This returnes the AttitudeRotationMatrix based on the flight stack that is
    * being run.  Note this is a **NOT A BLOCKING CALL**.  If there is no data,
    * the number of Attitude Rotation Matrix samples returned is 0.  The amount of buffering on the
    * adsp is 100 samples.  If the buffer is full, the oldest data is overwritten.
    * To get a blocking behaviour, you may call "GetData()" method first before calling
    * this method.
    * **NOTE** This can return a value of zero samples under the following conditions:
    *   * There are no AttitudeRotationMatrix samples
    *   * There is not flight stack running to generate the data
    *   * The implementation of the flight stack is not reporting the data to be
    *     sent to the Apps process. In this case contact the flight stack vendor/implementor
    *     to enable this.
    * @param sensor_attitude*
    *     A pointer to the sensor_rotation_matrix array to get the attitude from the adsp.
    * @param max_count
    *    The maximum size for the above array.
    * @param returned_sample_count
    *     the number of attitude_rotation_matrix samples returned.
    * @return
    *     0 = success
    *     other = failure;
    * @Note: The IMU data that is part of the data type is in the raw IMU Frame for the eagle board.
    **/
    int16_t getAttitudeBuffered( sensor_attitude* dataArray, int32_t max_count, int32_t* returned_sample_count )
    {
        if( !mSensorHandlePtr || !dataArray || !mFlightStackEnabled )
        {
            return -1;
        }
        uint8_t blockCall = 0;
        return sensor_imu_attitude_api_get_attitude( mSensorHandlePtr, dataArray, max_count, returned_sample_count, blockCall );
    }


    void addNewListener( ImuFrameListener* listener );

private:

    void ThreadMain();
    const float NORM_G = 9.80665f;
    bool mRunning; 
    bool mFlightStackEnabled;
    sensor_handle* mSensorHandlePtr;
    std::thread mImuPollThread;
    std::vector<ImuFrameListener*> mListeners;
};

#endif
