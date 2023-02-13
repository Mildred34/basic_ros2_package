import os
from ament_index_python.packages import get_package_share_directory
# launch ros module
from launch import LaunchDescription
from launch_ros.actions import Node

# Your launch file must contain this function: generate_launch_description(), 
# and must return a LaunchDescription object.

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('training_py'),
        'config',
        'configs.yaml'
        )

    # Your nodes 
    get_expert_pose_node = Node(
        package="training_py",
        executable="get_expert_pose_node",
        parameters = [config]
    )

    test_yaml_node = Node(
        package="training_py",
        executable="test_yaml_node",
        parameters = [config]
    )

    # Adding them to the launch description
    ld.add_action(get_expert_pose_node)
    ld.add_action(test_yaml_node)

    return ld