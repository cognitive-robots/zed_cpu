#include <memory>
#include <string>
#include <vector>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <zed_lib/sensorcapture.hpp>
#include <zed_lib/videocapture.hpp>


namespace zed_cpu {

  class ZedCameraNode {
    public:
      ZedCameraNode(std::shared_ptr<ros::NodeHandle> const& nh,
                    std::shared_ptr<image_transport::ImageTransport> const& it);

    protected:
      void initCamera(int const serial_number, sl_oc::video::RESOLUTION const resolution,
                      sl_oc::video::FPS const fps);
      void initImu(int const serial_number);
      void publishImages();
      void publishImu();

      std::shared_ptr<ros::NodeHandle> nh_;
      std::shared_ptr<image_transport::ImageTransport> it_;
      int serial_number_;
      int resolution_;
      int frame_rate_;
      int imu_rate_;
      std::string left_camera_frame_;
      std::string right_camera_frame_;
      std::string imu_frame_;
      std::unique_ptr<sl_oc::video::VideoCapture> video_capture_;
      std::unique_ptr<sl_oc::sensors::SensorCapture> imu_capture_;
      std::unique_ptr<camera_info_manager::CameraInfoManager> left_camera_info_manager_;
      std::unique_ptr<camera_info_manager::CameraInfoManager> right_camera_info_manager_;
      image_transport::CameraPublisher left_camera_pub_;
      image_transport::CameraPublisher right_camera_pub_;
      ros::Publisher imu_pub_;
      ros::Timer image_timer_;
      ros::Timer imu_timer_;
  };

}  // namespace zed_cpu