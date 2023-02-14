#include "my_robot_training/get_expert_pose_cpp.hpp"
// using moveit::planning_interface::MoveGroupInterface;

GetPose::GetPose()
: Node(
    "get_expert_pose_node_cpp",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
  count_(0)
{
  // Declare parameters ; Default value here
  declare_parameter("kinematic_only", false);
  declare_parameter("underactuation", false);
  declare_parameter("config_folder", "/training_py/data");
  declare_parameter("file_path", "/home/alex/additive_manufacturing/result/datasets");
  declare_parameter("object_name", "coude");

  // Get parameter values one by one
  auto param_underactuation = get_parameter("underactuation").as_bool();
  auto param_object_name = get_parameter("object_name").as_string();

  RCLCPP_INFO(logger, "Object name: %s", param_object_name.c_str());
  RCLCPP_INFO(logger, "Underactuation: %d", param_underactuation);

  static const string group_name = "robot_arm";

  // Moveit configuration
  // Create the MoveIt MoveGroup Interface
  static const string PLANNING_GROUP = "panda_arm";
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  executor.spin();

  auto move_group_interface =
    moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP);

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    RCLCPP_INFO(logger, "Planing OK, running in progress...");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  // timer_ = this->create_wall_timer(500ms, std::bind(&GetPose::timer_callback, this));
}

/**
*  @brief Timer who get published some msg everytime is called
*
*  @todo  .
*
*  @param .
*  @return .
*/
void GetPose::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GetPose>());
  rclcpp::shutdown();
  return 0;
}