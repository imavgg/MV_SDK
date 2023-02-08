/****************************************************************************
 *   Copyright (c) 2016 Ramakrishna Kintada. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#pragma once

// global/system includes.
#include <iostream>
#include <thread>
#include <vector>
#include <condition_variable>
#include <mutex>

// camera API Headers for the Snapdragon flight.
#include "camera.h"
#include "camera_parameters.h"

// Local
#include "SnapdragonCameraTypes.hpp"

namespace Snapdragon {
  class CameraManager;
}

//Class to support the OV7251 Camera.
class Snapdragon::CameraManager : public camera::ICameraListener
{
public:

  struct CameraFrameType
  {
    camera::ICameraFrame * frame;
    camera::ICameraFrame * frame_right;
    int64_t timestamp;
    int64_t frame_id;

    CameraFrameType()
    {
      frame = nullptr;
      frame_right = nullptr;
      timestamp = -1;
      frame_id = -1;
    }
  };

  /**
   * Contructor for the camera Manager.
   * @param params
   *   the camera parameters to use for the configuration.
   **/
  CameraManager( Snapdragon::CameraParameters * params );

  /**
   * Desctructor for the camera class.
   * make sure to close the camera istance if it opened.
   **/
  ~CameraManager()
  {
    Terminate();
  }

  /**
   * This function should be call first.
   * The configuration passed in the ctor, will be used for intializing the camera
   * @return int32_t
   *  0=success
   *  otherwise=failure.
   */
  int32_t Initialize();

  /**
   * Function to start the camera interface.
   * Note: Initialize Shall be called first.
   * @return int32_t
   *  0 = success;
   *  otherwise = failure.
   *  If Initialize is not called first, this function will fail.
   */
  int32_t Start();

  /**
   * Function to stop the camera.
   * @return int32_t
   *   0 = success;
   *  otherwise = failure.
   */
  int32_t Stop();

  /**
   * Function to allow for clean-up of resources allocated during initalize method.
   * @return int32_t
   *  0 = success
   * otherwise = failure.
   */
  int32_t Terminate();

  inline bool IsRunning()
  {
    return running_;
  }

  /**
   * Get the ID of the oldest frame in the queue
   * @returns
   *   frame id of oldest image in queue
   *   -1 if no frames are in queue
   */
  inline int64_t GetOldestFrameId()
  {
    std::lock_guard<std::mutex> lock( frame_mutex_ );
    int64_t rc = -1;
    if( next_frame_id_ > 0 ) {
      if( next_frame_id_ < camera_config_ptr_->num_image_buffers ) {
        rc = 0;
      }
      else {
        rc = next_frame_id_ - camera_config_ptr_->num_image_buffers;
      }
    }
    return rc;
  }

  /**
   * Get the ID of the newest frame in the queue
   * @returns
   *   frame id of newest image in queue
   *   -1 if no frames are in queue
   */
  inline int64_t GetNewestFrameId()
  {
    std::lock_guard<std::mutex> lock( frame_mutex_ );
    int64_t rc = -1;
    if( next_frame_id_ > 0 ) {
      rc = next_frame_id_ - 1;
    }
    return rc;
  }

  /**
   * Blocking call to copy image with id equal to frame_id from the queue into
   * given buffer.
   * If frame_id >= next frame to be read, function will block until
   * the image becomes available.
   * If frame_id has already been lost (overwritten in buffer), function
   * returns immediately with error code.
   *
   * @returns
   *   0 if requested frame is returned
   *  -1 if requested frame is already lost
   *  -2 if size is less than the size of the image being requested
   */
  int32_t GetNextImageData(int64_t* frame_id, uint64_t* timestamp_ns,
      uint32_t size, uint32_t* used, uint8_t* image_data);

  int32_t GetNextStereoImageData(int64_t* frame_id, uint64_t* timestamp_ns,
      uint32_t size, uint32_t* used,
      uint8_t* image_data_left, uint8_t* image_data_right);

  void SetManualExposure( float exposure );
  void SetManualGain( float gain );

  inline size_t GetImageSize() const
  {
    return image_size_bytes_;
  }

  inline double GetAvgFps() const
  {
    return fps_avg_;
  }

  inline float GetExposureTimeUs() const
  {
#ifdef QC_SOC_TARGET_APQ8074
    return static_cast<float>(exposure_setting_) * camera_config_ptr_->row_period_us;
#endif
#ifdef QC_SOC_TARGET_APQ8096
    return exposure_time_ns_/1000;
#endif
  }

  inline float GetExposureValue() const
  {
    float exposure_value = (float)(exposure_setting_ - camera_config_ptr_->min_exposure) /
        (float)(camera_config_ptr_->max_exposure - camera_config_ptr_->min_exposure);
    return exposure_value;
  }

  inline float GetGainValue() const
  {
    float gain_value = (float)(gain_setting_ - camera_config_ptr_->min_gain) /
        (float)(camera_config_ptr_->max_gain - camera_config_ptr_->min_gain);
    return gain_value;
  }

  //interface functions from the camera::ICameraListener
  virtual void onError();
  virtual void onPreviewFrame(camera::ICameraFrame* frame);
  virtual void onVideoFrame(camera::ICameraFrame* frame);

private: // private class methods.

  void UpdateGainAndExposure();

  int32_t GetImageData(int64_t* frame_id, uint64_t* timestamp_ns,
      uint32_t size, uint32_t* used,
      uint8_t* image_data, uint8_t* image_data_right = nullptr);

private: // class private data members.
  bool initialized_;
  bool running_;

  Snapdragon::CameraParameters* snap_camera_param_ptr_;
  Snapdragon::CameraConfig*     camera_config_ptr_;

  camera::ICameraDevice* camera_ptr_;
  camera::CameraParams   params_;
  camera::ImageSize      preview_size_;

  size_t   image_size_bytes_;
  int32_t  exposure_setting_;
  int32_t  gain_setting_;
  int32_t  exposure_target_;
  int32_t  gain_target_;
  float    fps_avg_;
  uint64_t timestamp_last_nsecs_;
  float    exposure_time_ns_;
  int64_t  next_frame_id_;
  uint32_t frame_q_write_index_;
  uint32_t frame_q_read_index_;

  std::mutex              frame_mutex_;
  std::condition_variable frame_cv_;

  std::vector<CameraFrameType> frame_queue_;
};
