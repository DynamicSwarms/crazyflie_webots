from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crazyflie_interfaces.msg import GenericLogData
from std_msgs.msg import Int16, Empty

from typing import List, Callable


class LogBlockServer:

    def __init__(self, node: Node, name: str):
        self.node = node
        self._log_block_start_callback = None
        self._log_block_stop_callback = None
        callback_group = MutuallyExclusiveCallbackGroup()
        qos_profile = 10
        namespace = "~/log/{}/".format(name)

        node.create_subscription(
            Int16,
            self._start_log_block,
            namespace + "start",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            Empty,
            self._stop_log_block,
            namespace + "stop",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        self.log_data_publisher = node.create_publisher(
            GenericLogData,
            namespace + "data",
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

    def set_log_block_start_callback(self, callback: Callable[[int],]) -> None:
        self._log_block_start_callback = callback

    def set_log_block_stop_callback(self, callback: Callable[[],]) -> None:
        self._log_block_stop_callback = callback

    def send_data(self, values: List[float]):
        msg = GenericLogData()
        msg.values = values
        self.log_data_publisher.publish(msg)

    def _start_log_block(self, msg):
        period_ms = msg.data
        if self._log_block_start_callback:
            return self._log_block_start_callback(period_ms)
        self.node.get_logger().warn(
            "Cannot start log block, server did not provide callback yet"
        )

    def _stop_log_block(self, msg):
        if self._log_block_stop_callback:
            return self._log_block_stop_callback()
        self.node.get_logger().warn(
            "Cannot stop log block, server didnt provide callback yet"
        )
