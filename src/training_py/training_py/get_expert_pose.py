#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_simulation_pkg.robotSimulation import RobotSimulation


class GetPose(Node):
    def __init__(self):
        # Création du noeud
        super().__init__('get_expert_pose_node')

        # Récupération de la config
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kinematic_only', False),
                ('underactuation', True),
                ('config_folder', "/training_py/data"),
                ('file_path', "/home/alex/additive_manufacturing/result/datasets"),
                ('object_name', "coude")
            ])

        # Test configuration
        param_underactuation = self.get_parameter("underactuation").get_parameter_value().bool_value
        param_object_name = self.get_parameter("object_name").get_parameter_value().string_value

        self.get_logger().info('Underactuation: {}'.format(param_underactuation))
        self.get_logger().info('Object name: {}'.format(param_object_name))

        self.robot_sim = RobotSimulation(param_object_name)

def main(args=None):
    rclpy.init(args=args)
    node = GetPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()