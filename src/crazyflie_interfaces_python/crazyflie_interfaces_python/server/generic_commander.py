from abc import ABC, abstractmethod

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from crazyflie_interfaces.msg import (
    NotifySetpointsStop,
    VelocityWorld,
    Hover,
    FullState,
    Position,
)

from typing import List


class GenericCommanderServer(ABC):
    def __init__(self, node: Node):
        callback_group = MutuallyExclusiveCallbackGroup()

        node.create_subscription(
            NotifySetpointsStop,
            "~/notify_setpoints_stop",
            self._notify_setpoints_stop,
            10,
            callback_group=callback_group,
        )
        node.create_subscription(
            VelocityWorld, "~/cmd_vel", self._cmd_vel, 10, callback_group=callback_group
        )
        node.create_subscription(
            Hover, "~/cmd_hover", self._cmd_hover, 10, callback_group=callback_group
        )
        node.create_subscription(
            FullState,
            "~/cmd_full_state",
            self._cmd_full_state,
            10,
            callback_group=callback_group,
        )
        node.create_subscription(
            Position,
            "~/cmd_position",
            self._cmd_position,
            10,
            callback_group=callback_group,
        )

    @abstractmethod
    def notify_setpoints_stop(self, remain_valid_millisecs: int) -> None:
        """Gets called if a notify setpoints stop command is received from ros

        Notify the crazyflie that streaming setpoints will end.

        Args:
            remain_valid_millisecs (int): _description_
        """
        pass

    @abstractmethod
    def velocity_world_setpoint(
        self, vx: float, vy: float, vz: float, yawrate: float
    ) -> None:
        """Gets called if a velocity world command is received from ros

        Flies crazyflie in with given velocities in world coordinates

        Args:
            vx (float): _description_
            vy (float): _description_
            vz (float): _description_
            yawrate (float): _description_
        """
        pass

    @abstractmethod
    def hover_setpoint(
        self, vx: float, vy: float, yawrate: float, z_distance: float
    ) -> None:
        """Gets called if a hover command is received from ros

        Crazyfie should hover in given height with given velocities

        Args:
            vx (float): _description_
            vy (float): _description_
            yawrate (float): _description_
            z_distance (float): _description_
        """
        pass

    @abstractmethod
    def full_state_setpoint(
        self,
        position: List[int],
        velocity: List[int],
        acceleration: List[int],
        orienation: List[int],
        rates: List[int],
    ) -> None:
        """TODO

        Args:
            position (List[int]): _description_
            velocity (List[int]): _description_
            acceleration (List[int]): _description_
            orienation (List[int]): _description_
            rates (List[int]): _description_
        """
        pass

    @abstractmethod
    def position_setpoint(self, x: float, y: float, z: float, yaw: float) -> None:
        """TODO

        Args:
            x (float): _description_
            y (float): _description_
            z (float): _description_
            yaw (float): _description_
        """
        pass

    def _notify_setpoints_stop(self, msg: NotifySetpointsStop) -> None:
        self.notify_setpoints_stop(msg.remain_valid_millisecs)

    def _cmd_vel(self, msg: VelocityWorld) -> None:
        self.velocity_world_setpoint(msg.vel.x, msg.vel.y, msg.vel.z, msg.yaw_rate)

    def _cmd_hover(self, msg: Hover) -> None:
        self.hover_setpoint(msg.vx, msg.vy, msg.yawrate, msg.z_distance)

    def _cmd_full_state(self, msg: FullState) -> None:
        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        velocity = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        acceleration = [msg.acc.x, msg.acc.y, msg.acc.z]
        orientation = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.y,
        ]
        rates = [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
        self.full_state_setpoint(position, velocity, acceleration, orientation, rates)

    def _cmd_position(self, msg: Position) -> None:
        self.position_setpoint(msg.x, msg.y, msg.z, msg.yaw)