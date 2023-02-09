#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class TestYAMLParams(Node):

    def __init__(self):
        super().__init__('test_yaml_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('bool_value', False),
                ('int_number', 1),
                ('float_number', 0.2),
                ('str_text', "wow"),
                ('bool_array', [True,False]),
                ('int_array', [1,2,3]),
                ('float_array', [0.1,0.2]),
                ('str_array', ["a","b"]),
                ('bytes_array', [0x01, 0xF1, 0xA2])
            ])

        param_int = self.get_parameter("int_number").get_parameter_value().integer_value

        self.get_logger().info('My log message {}'.format(param_int))

def main(args=None):
    rclpy.init(args=args)
    node = TestYAMLParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()