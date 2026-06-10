#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

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

    mode_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/sweeper_controller/commands", 10);

    duplo_count_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/duplo_counter_controller/commands", 10);
  }

private:
  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    SweeperMode mode = SweeperMode::Idle;
    bool duplo_count_requested = false;

    if (msg->buttons[0]) {
      mode = SweeperMode::Collect;
    }
    else if (msg->buttons[1]) {
      mode = SweeperMode::Dropoff;
    }
    else if (msg->buttons[2]) {
      duplo_count_requested = true;
    }

    // Publish duplo count request
    double duplo_count_value = duplo_count_requested ? 1.0 : 0.0;
    
    std_msgs::msg::Float64MultiArray duplo_msg;
    duplo_msg.data = {duplo_count_value};
    duplo_count_pub_->publish(duplo_msg);

    // Publish sweeper mode request
    double mode_value = static_cast<double>(mode);

    if (mode_value == last_mode_) {
      return;
    }

    std_msgs::msg::Float64MultiArray out;
    out.data = {mode_value};

    mode_pub_->publish(out);
    last_mode_ = mode_value;
  }

  double last_mode_ = -1.0;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr duplo_count_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyModeMapper>());
  rclcpp::shutdown();
  return 0;
}