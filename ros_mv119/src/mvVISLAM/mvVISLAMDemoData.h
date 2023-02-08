/***************************************************************************//**
@internal
   Copyright 2017 Qualcomm Technologies, Inc.  All rights reserved.
   Confidential & Proprietary.
*******************************************************************************/

// Structure for VISLAM sensor (accelerometer/gyro) data
struct mvVISLAM_SensorData
{
    float64_t x;
    float64_t y;
    float64_t z;
    int64_t time;
};


// Sample accelerometer data
const mvVISLAM_SensorData MVSampleApp1_mvVISLAM_AccelData[] = {
    { 5.310507, 0.584534, 8.204739, 625391010676335 },
};

// Sample gyro data
const mvVISLAM_SensorData MVSampleApp1_mvVISLAM_GyroData[] = {
    { 0.044741, -0.055394, 0.031958, 625391010676335 },
};

// Sample image data
const unsigned int ImageFrameLength = 460840;
//Using fake image data for now
