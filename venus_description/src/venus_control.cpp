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

// class jointSubscriber : public rclcpp::Node
// {
//     public:
//     jointSubscriber()
//     : Node("joint_subscriber")
//     {
//       subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
//       "joint_states", 10, std::bind(&jointSubscriber::jointStatesCallback, this, _1)); // No slash here
//     }

//     private:
//     void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) const
//     {
//       if (msg == NULL) return;
//       s_jointStatesList.push(*msg);
//       if (s_jointStatesList.size() >= 10)
//       {
//         s_jointStatesList.pop();
//         RCLCPP_WARN(this->get_logger(), "jointStatesCallback, pop");
//       }
//       RCLCPP_INFO(this->get_logger(), "I got a message!");
//     }
//     rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
// };

void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg == NULL) return;
  s_jointStatesList.push(*msg);
  if (s_jointStatesList.size() >= 10)
  {
    s_jointStatesList.pop();
    RCLCPP_WARN(rclcpp::get_logger("callback"), "jointStatesCallback, pop");
  }
  RCLCPP_INFO(rclcpp::get_logger("callback"), "I got a message!");
}

int main(int argc, char *argv[])
{
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<jointSubscriber>());
    // rclcpp::shutdown();

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("joint_subscriber");
    auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 100, jointStatesCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    // rclcpp::Rate loop_rate(10);.
    // while (rclcpp::ok())
    // {
    //   rclcpp::spin_some(node);
    //   loop_rate.sleep();
    //   if (!s_jointStatesList.empty())
    //   {
    //     sensor_msgs::msg::JointState::SharedPtr js_data;
    //     *js_data = s_jointStatesList.front();
    //     s_jointStatesList.pop();
    //     for(int i = 0; i < 6; i++)
    //     {
    //       // Since we have 2 actuators at joint 2 (the index here is the same as that in Rviz 2),
    //       // you have to deal with actuator No.2 and No.3,
    //       // making them rotating to opposite directions
    //       ActuatorController::UnifiedID actuator = uIDArray.at(i);
    //       RCLCPP_INFO(node->get_logger(), "jointstate_%d, %s, %f", i+1, js_data->name[i].c_str(), js_data->position[i]);
    //       pController->setPosition(actuator.actuatorID, RAD_TO_POS(js_data->position[i]));
    //     }
    //   }
    // }

    return 0;
}