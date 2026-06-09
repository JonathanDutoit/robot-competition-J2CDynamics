#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include "j2cdynamics_driver/arduino_hardware_interface.hpp"

class JoyModeMapper : public rclcpp::Node
{
public:
  JoyModeMapper() : Node("joy_mode_mapper")
  {
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&JoyModeMapper::onJoy, this, std::placeholders::_1));

    mode_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/robot_mode", 10);
  }

private:
  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    j2cdynamics_driver::SweeperMode mode = j2cdynamics_driver::SweeperMode::Idle;

    // Xbox mapping (adjust if needed)
    if (msg->buttons[0]) {        // A
      mode = j2cdynamics_driver::SweeperMode::Collect;
    }
    else if (msg->buttons[1]) {   // B
      mode = j2cdynamics_driver::SweeperMode::Dropoff;
    }
    else if (msg->buttons[2]) {   // X
      mode = j2cdynamics_driver::SweeperMode::Idle;
    }

    auto out = std_msgs::msg::Float64();
    out.data = static_cast<double>(mode);

    mode_pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mode_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyModeMapper>());
  rclcpp::shutdown();
  return 0;
}