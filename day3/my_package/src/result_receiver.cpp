#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>

class ResultReceiver : public rclcpp::Node
{
public:
  ResultReceiver() : Node("result_receiver")
  {
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "result", 10, std::bind(&ResultReceiver::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received result: %d", msg->data);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ResultReceiver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
