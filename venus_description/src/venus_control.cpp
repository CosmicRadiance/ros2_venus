#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <queue>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "actuatorcontroller.h"

#define DEG_TO_RAD(x) ((x)*M_PI / 180.0)
#define RAD_TO_DEG(x) ((x)*180.0 / M_PI)

#define STEERING_GEAR_RATIO 36

#define RAD_TO_POS(x) ((x / (2 * M_PI)) * STEERING_GEAR_RATIO)
#define POS_TO_RAD(x) ((x * (2 * M_PI)) / STEERING_GEAR_RATIO)

using std::placeholders::_1;

static std::queue<sensor_msgs::msg::JointState> s_jointStatesList;

class jointSubscriber : public rclcpp::Node
{
    public:
    jointSubscriber()
    : Node("joint_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&jointSubscriber::jointStatesCallback, this, _1));
    }

    private:
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) const
    {
      if (msg == NULL) return;
      s_jointStatesList.push(*msg);
      if (s_jointStatesList.size() >= 10)
      {
        s_jointStatesList.pop();
        RCLCPP_WARN(this->get_logger(), "jointStatesCallback, pop");
      }
      RCLCPP_INFO(this->get_logger(), "I got a message!");
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<jointSubscriber>());
    rclcpp::shutdown();
    return 0;
}