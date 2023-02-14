#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_simulation_pkg/RobotSimulation.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals;

class GetPose : public rclcpp::Node
{
    public:
        GetPose();

    private:
        void timer_callback();
        
        rclcpp::Logger const logger = rclcpp::get_logger("hello_moveit");
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
};


