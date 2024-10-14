#include <cstdlib>
#include <memory>

#include <ros/ros.h>

#include "zed_cpu.h"


int main(int argc, char * argv[]) {
  ros::init(argc, argv, "zed_camera");

  auto nh = std::make_shared<ros::NodeHandle>("~");
  auto it = std::make_shared<image_transport::ImageTransport>(*nh);

  zed_cpu::ZedCameraNode zed_camera_node {nh, it};
  ros::AsyncSpinner spinner {2};
  spinner.start();

  ros::waitForShutdown();
  return EXIT_SUCCESS;
}