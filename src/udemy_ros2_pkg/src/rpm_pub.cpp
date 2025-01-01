#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <chrono>
#include <functional>
#include <iostream>

using namespace std::chrono_literals;

const double DEFAULT_RPM_VALUE = 100.0;

class RpmPubNode : public rclcpp::Node
{
public:
  RpmPubNode() : Node("rpm_pub_node")
  {
    this->declare_parameter<double>("rpm_val", DEFAULT_RPM_VALUE);
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("rpm", 10);
    timer_ =this->create_wall_timer(
        1s,
        std::bind(&RpmPubNode::publish_rpm, this)
    );
    std::cout << "RPM Publisher Node is running..." << std::endl;

  }

  private:
    void publish_rpm()
    {
      auto message = std_msgs::msg::Float64();
      //this->get_parameter("rpm_val", rpm_val_param_);
      //message.data = rpm_val_param_;
      
      //rclcpp::Parameter 
      double rpm_val_param_object = DEFAULT_RPM_VALUE;
      this->get_parameter("rpm_val", rpm_val_param_object);
      //message.data = rpm_val_param_object.as_double();
      message.data = rpm_val_param_object;

      publisher_->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    //double rpm_val_param_ = DEFAULT_RPM_VALUE;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RpmPubNode>());
  rclcpp::shutdown();

  return 0;
}