from abc import ABC, abstractmethod

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Empty


from typing import List


class LoggingServer(ABC):
    def __init__(self, node: Node):
        callback_group = MutuallyExclusiveCallbackGroup()

        node.create_subscription(
            Empty,
            "~/download_logging_toc",
            self._download_toc,
            10,
            callback_group=callback_group,
        )

        node.create_subscription(
            Empty,
            "~/get_logging_toc_info",
            self._get_toc_info,
            10,
            callback_group=callback_group,
        )

    @abstractmethod
    def start_block(self, id: int, period: int) -> None:
        pass

    @abstractmethod
    def add_block(self, id: int, elements: List[tuple]) -> None:
        pass

    @abstractmethod
    def download_toc(self) -> None:
        pass

    @abstractmethod
    def get_toc_info(self) -> None:
        pass

    def _download_toc(self, msg: Empty) -> None:
        self.download_toc()

    def _get_toc_info(self, msg: Empty) -> None:
        self.get_toc_info()
