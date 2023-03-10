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
#include "SnapdragonCameraManager.hpp"
#include "SnapdragonCameraUtil.hpp"
#include "SnapdragonDebugPrint.h"

#include <fstream>
#include <iostream>
#include <string>
#include <chrono>

Snapdragon::CameraManager::CameraManager( Snapdragon::CameraParameters* params_ptr) {
  initialized_ = false;
  running_     = false;

  snap_camera_param_ptr_ = params_ptr;
  camera_config_ptr_     = &params_ptr->camera_config;

  camera_ptr_           = NULL;
  if( camera_config_ptr_->cam_type == CameraType::STEREO ) {
    preview_size_.width = camera_config_ptr_->pixel_width * 2;
  }
  else {
    preview_size_.width = camera_config_ptr_->pixel_width;
  }
  preview_size_.height  = camera_config_ptr_->pixel_height;
  image_size_bytes_     = camera_config_ptr_->pixel_height * camera_config_ptr_->memory_stride *1.5;
  exposure_setting_     = camera_config_ptr_->min_exposure
                          + camera_config_ptr_->exposure
                          * (camera_config_ptr_->max_exposure - camera_config_ptr_->min_exposure);

  gain_setting_         = camera_config_ptr_->min_gain + camera_config_ptr_->gain
                          * (camera_config_ptr_->max_gain - camera_config_ptr_->min_gain);

  exposure_target_      = exposure_setting_;
  gain_target_          = gain_setting_;
  fps_avg_              = 0;
  timestamp_last_nsecs_ = 0;
  exposure_time_ns_     = -1.0;

  next_frame_id_ = 0;
  frame_q_write_index_ = 0;
  frame_q_read_index_ = 0;

  for (unsigned int ii = 0; ii < camera_config_ptr_->num_image_buffers; ++ii)
  {
    CameraFrameType null_frame;
    frame_queue_.push_back( null_frame );
  }
}

int32_t Snapdragon::CameraManager::Initialize(){
  if (!initialized_) {
    int32_t cam_id;
    if( Snapdragon::FindCamera( camera_config_ptr_->cam_type, &cam_id ) != 0 ) {
      ERROR_PRINT( "Cannot Find Camera Id for Type: %d", camera_config_ptr_->cam_type );
      return -1;
    }
    int ret = camera::ICameraDevice::createInstance(cam_id, &camera_ptr_);
    if (ret != 0) {
      ERROR_PRINT("Could not open camera %d", cam_id);
      return ret;
    }
    else {
      INFO_PRINT("Opened camera %d Type: %d", cam_id, camera_config_ptr_->cam_type );
    }

    camera_ptr_->addListener(this);

    ret = params_.init(camera_ptr_);
    if (ret != 0) {
      ERROR_PRINT("Failed to initialize camera parameters.");
      camera::ICameraDevice::deleteInstance(&camera_ptr_);
      return ret;
    }

    // Check desired FPS against supported FPS values
    std::vector<camera::Range> preview_fps_ranges = params_.getSupportedPreviewFpsRanges();
    int fps_index = -1;
    for (unsigned int ii = 0; ii < preview_fps_ranges.size(); ++ii) {
      INFO_PRINT("Preview FPS range %d: [ %d, %d ]",
        ii, preview_fps_ranges[ii].min / 1000, preview_fps_ranges[ii].max / 1000);
      if (preview_fps_ranges[ii].max / 1000 == camera_config_ptr_->fps) {
        fps_index = static_cast<int>(ii);
      }
    }

    // Check desired preview size against supported sizes
    std::vector<camera::ImageSize> preview_sizes = params_.getSupportedPreviewSizes();
    int psize_index = -1;
    for (unsigned int ii = 0; ii < preview_sizes.size(); ++ii) {
      INFO_PRINT("Preview size %d: [ %d x %d ]",
        ii, preview_sizes[ii].width, preview_sizes[ii].height);
      if (preview_sizes[ii].width == preview_size_.width &&
          preview_sizes[ii].height == preview_size_.height) {
        psize_index = static_cast<int>(ii);
      }
    }

    // Print supported preview formats
    // Default format is YUV
    std::vector<std::string> preview_formats = params_.getSupportedPreviewFormats();
    for (unsigned int ii = 0; ii < preview_formats.size(); ++ii) {
      INFO_PRINT("Preview format %d: %s", ii, preview_formats[ii].c_str());
    }

    {
      std::lock_guard<std::mutex> lock( frame_mutex_ );

      if (fps_index != -1)
      {
        INFO_PRINT("Setting FPS to %d", camera_config_ptr_->fps);
        params_.setPreviewFpsRange(preview_fps_ranges[fps_index]);
      }
      else
      {
        ERROR_PRINT("Invalid FPS value of %d. Using camera default.",
          camera_config_ptr_->fps);
      }

      if (psize_index != -1)
      {
        INFO_PRINT("Setting preview size to %dx%d", preview_size_.width, preview_size_.height);
        params_.setPreviewSize(preview_size_);
      }
      else
      {
        ERROR_PRINT("Invalid preview size of %dx%d. Using camera default.",
          preview_size_.width, preview_size_.height);
      }

      if( camera_config_ptr_->cam_format == CameraFormat::RAW_FORMAT )
      {
        INFO_PRINT("Setting preview format to RAW_FORMAT");
        params_.set( "preview-format", "bayer-rggb" );
        params_.set( "picture-format", "bayer-mipi-10gbrg" );
        params_.set( "raw-size", "640x480" );
#ifdef QC_SOC_TARGET_APQ8096
        params_.setPreviewFormat(camera::FORMAT_RAW10);
#endif
      }
      else if( camera_config_ptr_->cam_format == CameraFormat::NV12_FORMAT )
      {
        INFO_PRINT("Setting preview format to NV12_FORMAT");
        params_.set( "preview-format", "nv12" );
      }
      else
      {
        INFO_PRINT("Using default preview format of YUV_FORMAT");
      }

#ifdef QC_SOC_TARGET_APQ8074
      char exposure_string[6];
      sprintf(exposure_string,"%d", exposure_setting_);
      char gain_string[6];
      sprintf(gain_string,"%d", gain_setting_);
      params_.set("qc-exposure-manual",exposure_string);
      params_.set("qc-gain-manual",gain_string);
#endif
#ifdef QC_SOC_TARGET_APQ8096
      params_.setManualExposure(exposure_setting_);
      params_.setManualGain(gain_setting_);
#endif

      // commit the camera parameters.
      if (params_.commit() != 0)
      {
        ERROR_PRINT("Error setting initial camera params.");
        return -1;
      }
    }

    initialized_ = true;
  }
  else {
    WARN_PRINT("CameraManager is already initialized.");
  }

  return 0;
}

int32_t Snapdragon::CameraManager::Terminate() {
  if( camera_ptr_ != nullptr ) {
    // remove this as a listener.
    camera_ptr_->removeListener( this );
    //stop the camera.
    Stop();
    //delete the camera ptr
    camera::ICameraDevice::deleteInstance(&camera_ptr_);
    camera_ptr_ = nullptr;
  }
  return 0;
}

int32_t Snapdragon::CameraManager::Start() {
  if (initialized_) {
    if (!running_) {
      int ret = camera_ptr_->startPreview();
      if (ret == 0) {
        running_ = true;
      }
      else {
        return ret;
      }
    }
    else {
      WARN_PRINT( "CameraManager is already started." );
    }
  }
  else {
    ERROR_PRINT( "CameraManager has not yet been initialized."  );
    return -1;
  }

  return 0;
}

int32_t Snapdragon::CameraManager::Stop() {
  if( running_ ) {
    camera_ptr_->stopPreview();
  }
  running_ = false;

  std::lock_guard<std::mutex> lock( frame_mutex_ );
  //increment the frame id so that the condition variable is woken up.
  next_frame_id_++;
  frame_cv_.notify_all();

  //if there are any frames that are not read. free the frames
  while( frame_q_read_index_ != frame_q_write_index_ ) {
    if( frame_queue_[frame_q_read_index_].frame != nullptr ) {
      frame_queue_[frame_q_read_index_].frame->releaseRef();
      CameraFrameType null_frame;
      frame_queue_[frame_q_read_index_] = null_frame;
      frame_q_read_index_++;
      frame_q_read_index_ = ( frame_q_read_index_ >= camera_config_ptr_->num_image_buffers)?0:frame_q_read_index_;
    }
  }
  return 0;
}

void Snapdragon::CameraManager::SetManualExposure( float exposure )
{
  exposure_target_ = static_cast<int>(camera_config_ptr_->min_exposure + exposure
      * (camera_config_ptr_->max_exposure - camera_config_ptr_->min_exposure));
}

void Snapdragon::CameraManager::SetManualGain( float gain )
{
  gain_target_ = static_cast<int>(camera_config_ptr_->min_gain + gain
      * (camera_config_ptr_->max_gain - camera_config_ptr_->min_gain));
}

void Snapdragon::CameraManager::UpdateGainAndExposure()
{

  static std::chrono::system_clock::time_point time_last;
  auto time_now = std::chrono::system_clock::now();
  auto diff = time_now - time_last;

  if( diff.count() > 0
      ||
      abs(exposure_target_ - exposure_setting_) > camera_config_ptr_->exposure_change_threshold
      ||
      abs(gain_target_ - gain_setting_) > camera_config_ptr_->gain_change_threshold
      &&
      (exposure_target_ != exposure_setting_ || gain_target_ != gain_setting_)
    ) {

        time_last = time_now;

#ifdef QC_SOC_TARGET_APQ8074
        char exposure_string[6];
        sprintf(exposure_string,"%d",exposure_target_);
        char gain_string[6];
        sprintf(gain_string,"%d",gain_target_);
        params_.set("qc-exposure-manual",exposure_string);
        params_.set("qc-gain-manual",gain_string);
#endif
#ifdef QC_SOC_TARGET_APQ8096
        params_.setManualExposure(exposure_target_);
        params_.setManualGain(gain_target_);
#endif
        params_.commit();

        // The exposure and gain take effect in the second frame
        // after the params are committed i.e. there is one frame in between
        // committing params and receiving a frame with the params applied
        static int exposure_temp = exposure_setting_;
        static int gain_temp = gain_setting_;
        exposure_setting_ = exposure_temp;
        gain_setting_ = gain_temp;
        exposure_temp = exposure_target_;
        gain_temp = gain_target_;
  }
}

void Snapdragon::CameraManager::onError()
{
  ERROR_PRINT("Camera error.");
  if( running_ ) {
    camera_ptr_->stopPreview();
  }
  running_ = false;
}

int32_t Snapdragon::CameraManager::GetNextImageData
(
  int64_t*  frame_id,
  uint64_t* timestamp_ns,
  uint32_t size,
  uint32_t* used,
  uint8_t* image_data
) {

  int32_t result = GetImageData( frame_id, timestamp_ns, size, used,
      image_data, nullptr );

  return result;

}

int32_t Snapdragon::CameraManager::GetNextStereoImageData
(
  int64_t*  frame_id,
  uint64_t* timestamp_ns,
  uint32_t size,
  uint32_t* used,
  uint8_t* image_data_left,
  uint8_t* image_data_right
) {

  int32_t result = GetImageData( frame_id, timestamp_ns, size, used,
      image_data_left, image_data_right );

  return result;
}

int32_t Snapdragon::CameraManager::GetImageData
(
  int64_t*  frame_id,
  uint64_t* timestamp_ns,
  uint32_t size,
  uint32_t* used,
  uint8_t* image_data,
  uint8_t* image_data_right
) {

  std::unique_lock<std::mutex> lock( frame_mutex_ );
  int32_t ret_code = 0;

  // wait for new frame if queue is empty.
  frame_cv_.wait( lock, [&]{ return (frame_q_read_index_ != frame_q_write_index_); } );

  if( !running_ ) {
    // the camera has stopped.  so return an error code.
    return  -1;
  }

  if( frame_queue_[ frame_q_read_index_].frame_id == -1 || frame_queue_[frame_q_read_index_].frame == nullptr ) {
    //this should not happen incorrect frame id;
    ERROR_PRINT( "FrameId at read_index(%d) is incorrect: ", frame_q_read_index_ );
    return -2;
  }

  if( size < image_size_bytes_ ) {
    ERROR_PRINT( "Insuffient image buffer size: given: %d expected: %d",
      size, image_size_bytes_ );
    return -3;
  }

  *frame_id = frame_queue_[ frame_q_read_index_].frame_id;
  *timestamp_ns = frame_queue_[ frame_q_read_index_].timestamp;
  *used = image_size_bytes_;

  //copy stereo camera data
  if( camera_config_ptr_->cam_type == CameraType::STEREO && image_data_right != nullptr ) {
    for( int i = 0; i < camera_config_ptr_->pixel_height; ++i ) {
      uint8_t* src_left = reinterpret_cast<uint8_t*>(
          frame_queue_[ frame_q_read_index_].frame->data + i * camera_config_ptr_->pixel_width * 2 );
      uint8_t* src_right = reinterpret_cast<uint8_t*>(
          src_left + camera_config_ptr_->pixel_width );
      memcpy( image_data + i * camera_config_ptr_->pixel_width, src_left, camera_config_ptr_->pixel_width );
      memcpy( image_data_right + i * camera_config_ptr_->pixel_width, src_right, camera_config_ptr_->pixel_width);
    }
  }
  //monocular image data
  else {
    memcpy( image_data, reinterpret_cast<uint8_t*>( frame_queue_[frame_q_read_index_].frame->data ), image_size_bytes_ );
  }

  //invalidate the entry in the queue;
  frame_queue_[frame_q_read_index_].frame->releaseRef();
  CameraFrameType null_frame;
  frame_queue_[frame_q_read_index_] = null_frame;

  // increment the read counter.
  frame_q_read_index_++;
  frame_q_read_index_ = ( frame_q_read_index_ >= camera_config_ptr_->num_image_buffers)?0:frame_q_read_index_;

  return 0;
}

void Snapdragon::CameraManager::onPreviewFrame(camera::ICameraFrame* frame)
{
  if (frame->timeStamp != timestamp_last_nsecs_)
  {
    //increment the frameid;
    int64_t old_frame_id = next_frame_id_;

    //set the image_size bytes information.
    image_size_bytes_ = frame->size;
    if( camera_config_ptr_->cam_type == CameraType::STEREO ) {
      image_size_bytes_ = image_size_bytes_ / 2;
    }
    next_frame_id_++;

    //now add the frame to the queue.
    {
      std::lock_guard<std::mutex> lock( frame_mutex_ );

      //check if the queue already has an valid frame.  If yes, release the frame to be used
      // by the camera pipeline.
      if( frame_queue_[ frame_q_write_index_ ].frame_id != -1 &&
        frame_queue_[ frame_q_write_index_ ].frame != nullptr ) {
        frame_queue_[ frame_q_write_index_ ].frame->releaseRef();
      }

      // add the frame to the queue.
      frame->acquireRef();
      CameraFrameType new_frame;
      new_frame.frame_id = next_frame_id_;
      new_frame.timestamp = frame->timeStamp;
      new_frame.frame = frame;
      frame_queue_[ frame_q_write_index_ ] = new_frame;

      //update the gain and exposure only one every 4 frames, as there a delay in taking the new
      // exposure parameters to take into effect.
#ifdef QC_SOC_TARGET_APQ8074
      if( next_frame_id_ > 0 && (next_frame_id_ % 4 ) == 0 ) {
        UpdateGainAndExposure();
      }
#endif
#ifdef QC_SOC_TARGET_APQ8096
      if( next_frame_id_ > 0 && (next_frame_id_ % 8 ) == 0 ) {
        UpdateGainAndExposure();
      }
#endif

      frame_q_write_index_++;
      frame_q_write_index_ = (frame_q_write_index_ >= camera_config_ptr_->num_image_buffers )?0:frame_q_write_index_;

      if( frame_q_write_index_ == frame_q_read_index_ ) {
        //queue is full, increment the read index;
        frame_q_read_index_++;
        frame_q_read_index_ = ( frame_q_read_index_ >= camera_config_ptr_->num_image_buffers)?0:frame_q_read_index_;
      }

      // notify the caller who is waiting on a frame.
      frame_cv_.notify_all();
    }

    // update the frame stats.
    uint64_t timestamp_nsecs = frame->timeStamp;
    uint32_t dt_nsecs = timestamp_nsecs - timestamp_last_nsecs_;
    timestamp_last_nsecs_ = timestamp_nsecs;
    fps_avg_ = ((fps_avg_ * old_frame_id) + (1e9 / dt_nsecs)) / (next_frame_id_);
#ifdef QC_SOC_TARGET_APQ8096
    exposure_time_ns_ = params_.getFrameExposureTime(frame);
#endif
  }
  else{
    WARN_PRINT( "Got duplicate image frame at timestamp: %lld frame_id: %llu", frame->timeStamp, next_frame_id_ );
  }
}

void Snapdragon::CameraManager::onVideoFrame(camera::ICameraFrame* frame)
{
  // Should never get this
  INFO_PRINT("Got video frame!");
}

