#!/usr/bin/env python3
from os import system
import rclpy
from rclpy.node import Node
import numpy as np
import pickle
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty


class PosesSaver(Node):
    def __init__(self):
        # Création du noeud
        super().__init__('save_poses_node')

        # Récupération de la config
        self.declare_parameters(
            namespace='',
            parameters=[
                ('file_path', "/home/alex/additive_manufacturing/result/datasets")
            ])

        # Init Registering pose subscriber
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'pose_registration',
            self.register_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("supervisor node listening report topic")

        self.file_path = self.get_parameter("file_path").get_parameter_value().string_value

        # init shutdown & save service
        self.shutdown_save_service = rospy.Service('shutdown_save', Empty, self.shutdown_save)

def main(args=None):
    rclpy.init(args=args)
    node = PosesSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()