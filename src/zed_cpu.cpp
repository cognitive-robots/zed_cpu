#include "zed_cpu.h"

#include <filesystem>
#include <memory>
#include <vector>

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <zed_lib/sensorcapture.hpp>
#include <zed_lib/videocapture.hpp>


namespace zed_cpu {

  ZedCameraNode::ZedCameraNode(std::shared_ptr<ros::NodeHandle> const& nh,
                               std::shared_ptr<image_transport::ImageTransport> const& it)
    : nh_(nh), it_(it) {
    // Parameter parsing
    if (nh_->hasParam("serial_number")) {
      nh_->getParam("serial_number", serial_number_);
    } else {
      ROS_FATAL_STREAM("No serial number specified!");
      ros::shutdown();
    }

    if (nh_->hasParam("resolution")) {
      nh_->getParam("resolution", resolution_);
    } else {
      resolution_ = static_cast<int>(sl_oc::video::RESOLUTION::HD720);
      ROS_WARN_STREAM("No resolution specified, defaulting to '" << resolution_ << "'.");
    }

    if (nh_->hasParam("frame_rate")) {
      nh_->getParam("frame_rate", frame_rate_);
    } else {
      frame_rate_ = static_cast<int>(sl_oc::video::FPS::FPS_30);
      ROS_WARN_STREAM("No frame rate specified, defaulting to '" << frame_rate_ << "'.");
    }

    std::string left_camera_calibration_url {};
    if (nh_->hasParam("left_camera_calibration_url")) {
      nh_->getParam("left_camera_calibration_url", left_camera_calibration_url);
    } else {
      ROS_FATAL_STREAM("No camera calibration URL for left camera specified!");
      ros::shutdown();
    }

    std::string right_camera_calibration_url {};
    if (nh_->hasParam("right_camera_calibration_url")) {
      nh_->getParam("right_camera_calibration_url", right_camera_calibration_url);
    } else {
      ROS_FATAL_STREAM("No camera calibration URL for right camera specified!");
      ros::shutdown();
    }

    if (nh->hasParam("left_camera_frame")) {
      nh_->getParam("left_camera_frame", left_camera_frame_);
    } else {
      left_camera_frame_ = "left_camera_optical_frame";
      ROS_WARN_STREAM("No left camera frame specified, defaulting to '" << left_camera_frame_ << "'.");
    }

    if (nh->hasParam("right_camera_frame")) {
      nh_->getParam("right_camera_frame", right_camera_frame_);
    } else {
      right_camera_frame_ = "right_camera_optical_frame";
      ROS_WARN_STREAM("No right camera frame specified, defaulting to '" << right_camera_frame_ << "'.");
    }

     if (nh_->hasParam("imu_rate")) {
      nh_->getParam("imu_rate", imu_rate_);
    } else {
      imu_rate_ = 200;
      ROS_WARN_STREAM("No IMU rate specified, defaulting to '" << imu_rate_ << "'.");
    }

    if (nh_->hasParam("imu_frame")) {
      nh_->getParam("imu_frame", imu_frame_);
    } else {
      imu_frame_ = "imu_frame";
      ROS_WARN_STREAM("No IMU frame specified, defaulting to '" << imu_frame_ << "'.");
    }

    initCamera(serial_number_, static_cast<sl_oc::video::RESOLUTION>(resolution_),
               static_cast<sl_oc::video::FPS>(frame_rate_));
    initImu(serial_number_);

    left_camera_pub_ = it_->advertiseCamera("left/image_raw", 1);
    right_camera_pub_ = it_->advertiseCamera("right/image_raw", 1);
    ros::NodeHandle left_nh {"left"};
    left_camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(left_nh, "camera/left", left_camera_calibration_url);
    ros::NodeHandle right_nh {"right"};
    right_camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(right_nh, "camera/right", right_camera_calibration_url);
    imu_pub_ = nh_->advertise<sensor_msgs::Imu>("imu_data", 1);
    image_timer_ = nh_->createTimer(ros::Duration(1.0/frame_rate_), [this](ros::TimerEvent const&) {
      this->publishImages();
    });
    imu_timer_ = nh_->createTimer(ros::Duration(1.0/imu_rate_), [this](ros::TimerEvent const&) {
      this->publishImu();
    });

    return;
  }

  void ZedCameraNode::initCamera(int const serial_number, sl_oc::video::RESOLUTION const resolution,
                                sl_oc::video::FPS const fps) {
    // Find device id from serial number
    int device_id {-1};
    std::filesystem::path const dev_path {"/dev/"};
    for (auto const& entry: std::filesystem::directory_iterator(dev_path)) {
        std::string const filename {entry.path().filename().string()};
        sl_oc::video::VideoParams test_params {};
        test_params.verbose = sl_oc::VERBOSITY::NONE;
        if (filename.find("video") == 0) {
          auto test_capture {std::make_unique<sl_oc::video::VideoCapture>(test_params)};
          int const test_device_id {std::stoi(filename.substr(5))};
          test_capture->initializeVideo(test_device_id);
          auto const test_serial_number {test_capture->getSerialNumber()};
          if (test_serial_number == serial_number) {
            device_id = test_device_id;
            ROS_INFO_STREAM("Found device with serial number " << serial_number << "!");
            break;
          }
        }
    }
    if (device_id == -1) {
      ROS_ERROR_STREAM("Could not find device with serial number " << serial_number << "!");
      ros::shutdown();
      return;
    }

    sl_oc::video::VideoParams params {};
    params.res = resolution;
    params.fps = fps;
    params.verbose = sl_oc::VERBOSITY::ERROR;
    video_capture_ = std::make_unique<sl_oc::video::VideoCapture>(params);
    if (!video_capture_->initializeVideo(device_id)) {
      ROS_ERROR_STREAM("Failed to open camera video capture on device '" << device_id << "'!");
      ros::shutdown();
      return;
    }

    ROS_INFO_STREAM("Connected to device '" << device_id << "' with device name " << video_capture_->getDeviceName()
                    << " and serial number " << video_capture_->getSerialNumber());
    return;
  }

  void ZedCameraNode::initImu(int const serial_number) {
    imu_capture_ = std::make_unique<sl_oc::sensors::SensorCapture>(sl_oc::VERBOSITY::ERROR);

    std::vector<int> const devices {imu_capture_->getDeviceList()};
    if (devices.size() == 0) {
      ROS_FATAL_STREAM("No available ZED 2, ZED 2i or ZED Mini cameras found!");
      ros::shutdown();
      return;
    }

    if (!imu_capture_->initializeSensors(serial_number)) {
      ROS_FATAL_STREAM("Failed to initialize IMU of device " << serial_number);
      ros::shutdown();
      return;
    }

    std::uint16_t major_version {};
    std::uint16_t minor_version {};
    imu_capture_->getFirmwareVersion(major_version, minor_version);
    ROS_INFO_STREAM("Connected to IMU of device " << serial_number << " with firmware version: " << 
                    major_version << "." << minor_version);
    return;
  }

  void ZedCameraNode::publishImages() {
    sl_oc::video::Frame const frame {video_capture_->getLastFrame()};

    // We had the issue previously that when starting the driver sometimes just green frames would be shown
    // with a timestamp of 0
    if (frame.data != nullptr && frame.timestamp != 0) {
      cv::Mat const frame_yuv {cv::Mat(frame.height, frame.width, CV_8UC2, frame.data)};
      cv::Mat frame_bgr {};
      cv::cvtColor(frame_yuv, frame_bgr, cv::COLOR_YUV2BGR_YUYV);

      cv::Mat const left_img {frame_bgr(cv::Rect(0, 0, frame_bgr.cols / 2, frame_bgr.rows))};
      cv::Mat const right_img {frame_bgr(cv::Rect(frame_bgr.cols / 2, 0, frame_bgr.cols / 2, frame_bgr.rows))};

      ros::Time stamp {};
      stamp.fromNSec(frame.timestamp);

      std_msgs::Header left_img_header {};
      left_img_header.stamp = stamp;
      left_img_header.frame_id = left_camera_frame_;
      sensor_msgs::ImagePtr const left_msg  {cv_bridge::CvImage(left_img_header, "bgr8", left_img).toImageMsg()};

      std_msgs::Header right_img_header {};
      right_img_header.stamp = stamp;
      right_img_header.frame_id = right_camera_frame_;
      sensor_msgs::ImagePtr const right_msg {cv_bridge::CvImage(right_img_header, "bgr8", right_img).toImageMsg()};

      sensor_msgs::CameraInfo const left_camera_info_msg {left_camera_info_manager_->getCameraInfo()};
      sensor_msgs::CameraInfo const right_camera_info_msg {right_camera_info_manager_->getCameraInfo()};
      left_camera_pub_.publish(*left_msg, left_camera_info_msg);
      right_camera_pub_.publish(*right_msg, right_camera_info_msg);
    } else {
      ROS_WARN_STREAM("Failed to read camera frame!");
    }
    return;
  }

  void ZedCameraNode::publishImu() {
    sl_oc::sensors::data::Imu const imu_data {imu_capture_->getLastIMUData(2500)}; // Timeout of 2.5ms

    if (imu_data.valid == sl_oc::sensors::data::Imu::NEW_VAL) {
      sensor_msgs::Imu imu_msg {};
      ros::Time stamp {};
      stamp.fromNSec(imu_data.timestamp);
      imu_msg.header.stamp = stamp;
      imu_msg.header.frame_id = imu_frame_;

      imu_msg.linear_acceleration.x = imu_data.aX;
      imu_msg.linear_acceleration.y = imu_data.aY;
      imu_msg.linear_acceleration.z = imu_data.aZ;

      imu_msg.angular_velocity.x = imu_data.gX;
      imu_msg.angular_velocity.y = imu_data.gY;
      imu_msg.angular_velocity.z = imu_data.gZ;

      imu_pub_.publish(imu_msg);
    } else {
      ROS_WARN_STREAM("Failed to read IMU measurement!");
    }
    return;
  }

}  // namespace zed_cpu