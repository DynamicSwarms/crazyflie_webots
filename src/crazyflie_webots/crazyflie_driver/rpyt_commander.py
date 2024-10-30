from rclpy.node import Node
from crazyflie_interfaces_python.server.rpyt_commander import RPYTCommanderServer

from typing import List, Callable


class WebotsRPYTCommander(RPYTCommanderServer):
    def __init__(self, ros_node: Node):
        super().__init__(ros_node)

    def setpoint(self, roll: float, pitch: float, yawrate: float, thrust: int) -> None:
        return super().setpoint(roll, pitch, yawrate, thrust)
