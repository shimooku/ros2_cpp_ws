#include "rclcpp/rclcpp.hpp"
#include "udemy_ros2_pkg/srv/turn_camera.hpp"

#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

typedef udemy_ros2_pkg::srv::TurnCamera TurnCameraSrv;

class TurnCameraServiceNode : public rclcpp::Node
{
  public:
    TurnCameraServiceNode(std::string exe_dir) : Node("turn_camera_service_node")
    {
      ws_dir_ = get_ws_dir(exe_dir);
      service_server_=this->create_service<TurnCameraSrv>
      (
        "turn_camera",
        std::bind
        (
          &TurnCameraServiceNode::get_camera_image, 
          this,
          std::placeholders::_1, std::placeholders::_2
        )
      );
      std::cout << "Turn Camera Service is running..." << std::endl;
    }

  private:
    void get_camera_image
    (
      TurnCameraSrv::Request::SharedPtr request, 
      TurnCameraSrv::Response::SharedPtr response
    )
    {
      float closest_num = available_angles_[0];
      float angle_diff;
      float smallest_angle = std::abs(request->degree_turn - available_angles_[0]);

      for (int i = 0; i < 5; i++)
      {
        angle_diff = std::abs(request->degree_turn - available_angles_[i]);

        if (angle_diff < smallest_angle){
          smallest_angle = angle_diff;
          closest_num = available_angles_[i];
        }
      }

      std::string image_path = 
        ws_dir_ + "src/udemy_ros2_pkg/images/" + std::to_string((int)closest_num) + ".png";

      std::cout << image_path << std::endl;

      auto image = cv::imread(image_path);
      auto image_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

      response->camera_image = *image_ptr; // Copy....really....i wonder if it can provide enough speed.      
    }

    std::string get_ws_dir(std::string exe_dir)
    {
      std::string::size_type substr_index = exe_dir.find("install");
      return exe_dir.substr(0, substr_index);
    }

    rclcpp::Service<TurnCameraSrv>::SharedPtr service_server_;
    const float available_angles_[5] {-30, -15, 0, 15, 30};
    std::string ws_dir_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurnCameraServiceNode>(argv[0]));
  rclcpp::shutdown();

  return 0;
}