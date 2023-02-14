#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from .robot_simulation_pkg.robotSimulation import RobotSimulation
import moveit_commander
from std_msgs.msg import Float64MultiArray, Float64
from std_srvs.srv import Empty, SetBool


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
        
        group_name = "robot_arm"
        # Moveit configuration


        # init hand control
        self.hand_target_cli = self.create_client(HandTarget, 'hand_target')

        while not self.hand_target_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('hand_target service not available, waiting again...')

        if underactuation:
            self.finger_open_bound = -0.43
            self.finger_close_bound = 1.25
        else:
            self.finger_open_bound = 0
            self.finger_close_bound = 0.31

        self.hand_target(target=[self.finger_open_bound, self.finger_open_bound, self.finger_open_bound, 0])
        self.get_logger().debug("get_expert_node connected to hand controller")

        # init get hand pose service
        self.get_hand_pose_service = self.create_service(Empty, 'get_hand_pose', self.get_hand_pose)

        # init pose registration topic
        self.pose_registration = rospy.Publisher('pose_registration', Float64MultiArray, queue_size=10)

        # init object pose service
        self.set_object_pose_service =  self.create_service(SetPoseType, 'set_object_pose', self.set_object_pose)

        # init abduction angle service
        self.set_abduction_service = self.create_service(SetPoseType, 'set_abduction_angle', self.set_abduction_angle)

        # init close finger service
        self.close_finger_service = self.create_service(SetBool, 'close_finger', self.close_finger)

    def get_hand_pose(self, req):   
        pass

    def set_object_pose(self, req):
        pass

    def set_abduction_angle(self, req):
        pass

    def close_finger(self, req):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GetPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()