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

        # File path to save contact points
        object_name = self.get_parameter("object_name").get_parameter_value().string_value
        self.file_path = self.get_parameter("file_path").get_parameter_value().string_value + "/" \
            + "object_name" + "/" + "dataset_" + object_name + ".pkl"

        # init poses_list:
        self.poses_list = []

        # init shutdown & save service
        self.shutdown_save_service = self.create_service(Empty, 'shutdown_save', self.shutdown_save)

    def register_callback(self, data):
        self.poses_list.append(list(data.data))
        self.get_logger().warn("number of registered poses: %s", len(self.poses_list))

    def shutdown_save(self, req):
        poses_array = np.array(self.poses_list)

        with open(self.file_path, 'wb') as file:
            pickle.dump(poses_array, file)

        raise SystemExit 

        return []

def main(args=None):
    rclpy.init(args=args)
    node = PosesSaver()

    try: 
        rclpy.spin(node)
    except SystemExit:
        rclpy.get_logger("Quitting").info("End of pose registration")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()