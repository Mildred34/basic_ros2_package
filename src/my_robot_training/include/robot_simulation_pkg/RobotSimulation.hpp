#pragma once
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;

class RobotSimulation
{
public:
  RobotSimulation(string objet);
  ~RobotSimulation() = default;

private:
  string object_name;
};