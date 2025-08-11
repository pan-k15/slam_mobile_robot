#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdVelRelay : public rclcpp::Node
{
public:
  CmdVelRelay()
  : Node("cmd_vel_relay")
  {
    // Publisher to /diff_cont/cmd_vel_unstamped
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diff_cont/cmd_vel_unstamped", 10);

    // Subscriber to /cmd_vel
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelRelay::cmdVelCallback, this, std::placeholders::_1));
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Republish the incoming Twist message
    pub_->publish(*msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelRelay>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
