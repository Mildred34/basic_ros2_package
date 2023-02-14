#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>

using namespace std;

class RobotSimulation
{
    public:
        RobotSimulation(string objet);
        ~RobotSimulation() = default;

    private:
        string object_name;

};