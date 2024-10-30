from rclpy.node import Node
from crazyflie_interfaces_python.server.high_level_commander import (
    HighLevelCommanderServer,
)
from typing import List, Callable


class WebotsHighLevelCommander(HighLevelCommanderServer):

    def __init__(
        self,
        ros_node: Node,
        set_target: Callable[[List[float]], None],
        get_position: Callable[[], List[float]],
    ):
        super().__init__(ros_node)
        self.set_target = set_target
        self.get_position = get_position

    def set_group_mask(self, group_mask: float) -> None:
        return super().set_group_mask(group_mask)

    def takeoff(
        self,
        group_mask: int,
        height: float,
        yaw: float,
        use_current_yaw: bool,
        duration_seconds: float,
    ) -> None:
        pos = self.get_position()
        self.set_target([pos[0], pos[1], height])

    def land(
        self,
        group_mask: int,
        height: float,
        yaw: float,
        use_current_yaw: bool,
        duration_seconds: float,
    ) -> None:
        pos = self.get_position()
        self.set_target([pos[0], pos[1], height + 0.02])

    def stop(self, group_mask: int) -> None:
        return super().stop(group_mask)

    def go_to(
        self,
        group_mask: int,
        relative: bool,
        linear: bool,
        x: float,
        y: float,
        z: float,
        yaw: float,
        duration_seconds: float,
    ) -> None:
        self.set_target([x, y, z])
