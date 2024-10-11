#include <memory>
#include <string>
#include <vector>

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
      void initCamera(int const device_id, sl_oc::video::RESOLUTION const resolution,
                       sl_oc::video::FPS const fps);
      void initImu(int const serial_number);
      void publishImages();
      void publishImu();

      std::shared_ptr<ros::NodeHandle> nh_;
      std::shared_ptr<image_transport::ImageTransport> it_;
      int device_id_;
      int serial_number_;
      int resolution_;
      int frame_rate_;
      int imu_rate_;
      std::string left_camera_frame_;
      std::string right_camera_frame_;
      std::string imu_frame_;
      std::unique_ptr<sl_oc::video::VideoCapture> cap_;
      std::unique_ptr<sl_oc::sensors::SensorCapture> sens_;
      image_transport::Publisher left_image_pub_;
      image_transport::Publisher right_image_pub_;
      ros::Publisher imu_pub_;
      ros::Timer image_timer_;
      ros::Timer imu_timer_;
  };

}  // namespace zed_cpu