#include "my_robot_training/get_expert_pose_cpp.hpp"
// using moveit::planning_interface::MoveGroupInterface;

GetPose::GetPose()
: Node(
    "get_expert_pose_node_cpp",
    rclcpp::NodeOptions()), // .automatically_declare_parameters_from_overrides(true)
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
  spinner = std::thread([&executor]() { executor.spin(); });
  // executor.spin();

  // Need the movegroup node that provides the robot description
  auto move_group_interface =
    moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP);

  // Construct and initialize MoveItVisualTools
  // "panda_link0" base link of panda_arm
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      move_group_node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closures for visualization
  // lambda functions use later
  auto const draw_title = [&moveit_visual_tools](auto text) {

    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();

    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };

  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };

  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup(
          "panda_arm")](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };


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
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
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
  auto node = std::make_shared<GetPose>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  node->spinner.join();
  return 0;
}