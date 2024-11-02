from rclpy.node import Node
from crazyflie_interfaces_python.server.generic_commander import GenericCommanderServer


from typing import List, Callable


class WebotsGenericCommander(GenericCommanderServer):
    def __init__(
        self,
        ros_node: Node,
        set_target: Callable[[List[float]], None],
        get_position: Callable[[], List[float]],
    ):
        super().__init__(ros_node)
        self.set_target = set_target
        self.get_position = get_position

    def notify_setpoints_stop(self, remain_valid_millisecs: int) -> None:
        return super().notify_setpoints_stop(remain_valid_millisecs)

    def velocity_world_setpoint(
        self, vx: float, vy: float, vz: float, yawrate: float
    ) -> None:
        return super().velocity_world_setpoint(vx, vy, vz, yawrate)

    def hover_setpoint(
        self, vx: float, vy: float, yawrate: float, z_distance: float
    ) -> None:
        self.set_target(self.get_position())

    def full_state_setpoint(
        self,
        position: List[int],
        velocity: List[int],
        acceleration: List[int],
        orienation: List[int],
        rates: List[int],
    ) -> None:
        self.set_target(position)

    def position_setpoint(self, x: float, y: float, z: float, yaw: float) -> None:
        self.set_target([x, y, z])
