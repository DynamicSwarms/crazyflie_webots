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
        if relative:
            pos = self.get_position()
            x += pos[0]
            y += pos[1]
            z += pos[2]
        self.set_target([x, y, z])

    def start_trajectory(
        self,
        group_mask: int,
        trajectory_id: int,
        timescale: float,
        reversed: bool,
        relative: bool,
    ) -> None:
        return super().start_trajectory(
            group_mask, trajectory_id, timescale, reversed, relative
        )

    def define_trajectory(self, trajectory_id: int, piece_offset: int) -> None:
        return super().define_trajectory(trajectory_id, piece_offset)

    def takeoff_with_velocity(
        self,
        group_mask: int,
        height: float,
        height_is_relative: bool,
        yaw: float,
        use_current_yaw: bool,
        velocity: float,
    ) -> None:
        return super().takeoff_with_velocity(
            group_mask, height, height_is_relative, yaw, use_current_yaw, velocity
        )

    def land_with_velocity(
        self,
        group_mask: int,
        height: float,
        height_is_relative: bool,
        yaw: float,
        use_current_yaw: bool,
        velocity: float,
    ) -> None:
        return super().land_with_velocity(
            group_mask, height, height_is_relative, yaw, use_current_yaw, velocity
        )

    def spiral(
        self,
        group_mask: int,
        sidways: bool,
        clockwise: bool,
        phi: float,
        r0: float,
        rf: float,
        dz: float,
        duration_seconds: float,
    ) -> None:
        return super().spiral(
            group_mask, sidways, clockwise, phi, r0, rf, dz, duration_seconds
        )
