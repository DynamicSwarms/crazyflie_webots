from typing import List, Dict, Callable
from rclpy.node import Node
from rclpy.timer import Timer
from crazyflie_interfaces_python.server.logging import LoggingServer
from crazyflie_interfaces_python.server.logblock import LogBlockServer

from crazyflie_interfaces.msg import GenericLogData


class WebotsLogging(LoggingServer):

    def __init__(self, node: Node, available_logging_variables: dict):
        super().__init__(node)
        self.node = node
        self.available_logging_variables = available_logging_variables

        self.next_id: int = 0
        self.log_blocks: Dict[int, LogBlockServer] = {}
        self.block_callbacks: Dict[int, List[Callable[[None], float]]] = {}
        self.timers: Dict[int, Timer] = {}

    def create_log_block(self, variables: List[str], log_block: LogBlockServer) -> None:
        log_block_id = self.next_id

        self.log_blocks[log_block_id] = log_block

        self.block_callbacks[log_block_id] = []
        for variable in variables:
            if variable in self.available_logging_variables.keys():
                self.block_callbacks[log_block_id].append(
                    self.available_logging_variables[variable]
                )

        log_block.set_log_block_start_callback(
            lambda period_ms, log_block_id=log_block_id: self.start_block(
                log_block_id, period_ms
            )
        )
        log_block.set_log_block_stop_callback(
            lambda log_block_id=log_block_id: self.stop_block(log_block_id)
        )
        self.next_id += 1

    def start_block(self, id: int, period_ms: int) -> None:
        self.timers[id] = self.node.create_timer(
            timer_period_sec=float(period_ms) / 1000.0,
            callback=lambda id=id: self.send_block_data(id),
        )

    def stop_block(self, id: int) -> None:
        self.timers[id].destroy()

    def send_block_data(self, id: int):
        values = []
        for callback_function in self.block_callbacks[id]:
            values.append(callback_function())
        self.log_blocks[id].send_data(values)

    def download_toc(self) -> None:
        return super().download_toc()

    def get_toc_info(self) -> None:
        self.node.get_logger().info(
            "Currently available logging variables in webots simulation: "
            + str(self.available_logging_variables.keys())
        )
