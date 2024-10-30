from typing import List
from rclpy.node import Node

from crazyflie_interfaces_python.server.parameters import ParametersServer
from rclpy.parameter import Parameter


class WebotsParameters(ParametersServer):
    def __init__(self, node: Node):
        super().__init__(node)

    def download_toc(self) -> None:
        return super().download_toc()

    def get_toc_info(self) -> None:
        return super().get_toc_info()

    def set_parameter(self, group: str, name: str, value: Parameter.Type):
        return super().set_parameter(group, name, value)
