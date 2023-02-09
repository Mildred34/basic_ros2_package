import sys
from os import path
import rclpy

class RobotSimulation:
    def __init__(self,  object_name="coude"):
        self.object_name = object_name