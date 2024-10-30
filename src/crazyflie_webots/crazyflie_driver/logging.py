from typing import List
from rclpy.node import Node
from crazyflie_interfaces_python.server.logging import LoggingServer

from crazyflie_interfaces.msg import GenericLogData


class WebotsLogging(LoggingServer):

    def __init__(self, node: Node, available_logging_variables: dict):
        super().__init__(node)
        self.node = node
        self.available_logging_variables = available_logging_variables
        self.blocks = {}
        self.block_publishers = {}

    def start_block(self, id: int, period: int, name: str) -> None:
        self.block_publishers[id] = self.node.create_publisher(
            GenericLogData, "~/{}".format(name), 10
        )
        self.node.create_timer(
            float(period) / 1000, lambda id=id: self.publish_block(id)
        )

    def add_block(self, id: int, elements: List[dict]) -> None:
        self.blocks[id] = []
        for element in elements:
            variable = "{}.{}".format(element["group"], element["name"])
            if variable in self.available_logging_variables.keys():
                self.blocks[id].append(self.available_logging_variables[variable])

    def publish_block(self, id: int) -> None:
        msg = GenericLogData()
        for callback_function in self.blocks[id]:
            msg.values.append(callback_function())
        self.block_publishers[id].publish(msg)

    def download_toc(self) -> None:
        return super().download_toc()

    def get_toc_info(self) -> None:
        return super().get_toc_info()
