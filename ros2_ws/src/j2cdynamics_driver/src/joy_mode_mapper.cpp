#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>

enum class SweeperMode : int
{
  Idle = 0,
  Collect = 1,
  Dropoff = 2,
  Fault = 3
};

class JoyModeMapper : public rclcpp::Node
{
public:
  JoyModeMapper() : Node("joy_mode_mapper")
  {
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&JoyModeMapper::onJoy, this, std::placeholders::_1));

    mode_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/sweeper_mode_controller/commands", 10);
  }

private:
  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    SweeperMode mode = SweeperMode::Idle;

    // Xbox mapping (adjust if needed)
    if (msg->buttons[0]) {        // A
      mode = SweeperMode::Collect;
    }
    else if (msg->buttons[1]) {   // B
      mode = SweeperMode::Dropoff;
    }
    else if (msg->buttons[2]) {   // X
      mode = SweeperMode::Idle;
    }

    auto out = std_msgs::msg::Float64();
    out.data = static_cast<double>(mode);

    if (out.data != last_mode_) {
      mode_pub_->publish(out);
      last_mode_ = out.data;
    }
  }

  double last_mode_ = -1.0; // Initialize to an invalid mode

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