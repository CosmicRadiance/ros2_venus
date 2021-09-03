#include <iostream>
#include <chrono>
// #include <functional>
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

// using std::placeholders::_1;

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
    // rclcpp::spin(node);
    // rclcpp::shutdown();

    // Initialize the controller
    ActuatorController* pController = ActuatorController::initController();
    // ec Define an error type, ec==0x00 means no error,
    // ec will be passed to pcontroller-> lookupActuators(ec) by reference,
    // when the error occurs, ec value will be modified by SDK to the corresponding error code
    Actuator::ErrorsDefine ec;
    // Find the connected actuators and return the UnifiedID of all actuators found.
    std::vector<ActuatorController::UnifiedID> uIDArray = pController->lookupActuators(ec);
    // If the size of the uIDArray is greater than zero, the connected actuators have been found
    if (uIDArray.size() > 0)
    {
        RCLCPP_INFO(node->get_logger(), "Found %d actuators!", uIDArray.size());
        if (pController->enableActuatorInBatch(uIDArray))
            RCLCPP_INFO(node->get_logger(), "All actuators have been enabled successfully!");
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to enable actuators...");
            return -1;
        }
        for(size_t k = 0; k < uIDArray.size(); k++)
        {
            ActuatorController::UnifiedID actuator = uIDArray.at(k);
            pController->activateActuatorMode(actuator.actuatorID, Actuator::Mode_Profile_Pos);
            RCLCPP_INFO(node->get_logger(),
            "Set the position of actuator %d to zero, be careful.", actuator.actuatorID);
            pController->setPosition(actuator.actuatorID, 0);
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Connected error code: %x", ec);
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), "Initialized!");
    
    rclcpp::Rate loop_rate(50);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (!s_jointStatesList.empty())
        {
            sensor_msgs::msg::JointState js_data;
            js_data = s_jointStatesList.front();
            s_jointStatesList.pop();
            for(int i = 0; i < 6; i++)
            {
                // Since we have 2 actuators at joint 2 (the index here is the same as that in Rviz 2),
                // you have to deal with actuator No.2 and No.3,
                // making them rotating to opposite directions
                ActuatorController::UnifiedID actuator = uIDArray.at(i);
                // Actual movement at joint 1, 3, 4 are opposite
                if (i < 2)
                {
                    RCLCPP_INFO(node->get_logger(),
                    "jointstate_%d, %s, %f", i+1, js_data.name[i].c_str(), js_data.position[i]);
                    pController->setPosition(actuator.actuatorID, RAD_TO_POS(js_data.position[i]));
                }
                else if (i == 2)
                {
                    // RCLCPP_INFO(node->get_logger(),
                    // "jointstate_%d, %s, %f", i, js_data.name[i-1].c_str(), js_data.position[i-1]);
                    pController->setPosition(actuator.actuatorID, -1*RAD_TO_POS(js_data.position[i-1]));
                }
                else
                {
                    RCLCPP_INFO(node->get_logger(),
                    "jointstate_%d, %s, %f", i, js_data.name[i-1].c_str(), js_data.position[i-1]);
                    pController->setPosition(actuator.actuatorID, RAD_TO_POS(js_data.position[i-1]));
                }
            }
        }
        loop_rate.sleep();
    }

    return 0;
}