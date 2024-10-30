from abc import ABC, abstractmethod

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crazyflie_interfaces.msg import (
    SetGroupMask,
    Takeoff,
    Land,
    Stop,
    GoTo,
    StartTrajectory,
    UploadTrajectory,
    TakeoffWithVelocity,
    LandWithVelocity,
    Spiral,
)


class HighLevelCommanderServer(ABC):
    def __init__(self, node: Node):
        callback_group = MutuallyExclusiveCallbackGroup()

        node.create_subscription(
            SetGroupMask,
            "~/set_group_mask",
            self._set_group_mask,
            10,
            callback_group=callback_group,
        )
        node.create_subscription(
            Takeoff, "~/takeoff", self._takeoff, 10, callback_group=callback_group
        )
        node.create_subscription(
            Land, "~/land", self._land, 10, callback_group=callback_group
        )

        node.create_subscription(
            Stop, "~/stop", self._stop, 10, callback_group=callback_group
        )

        node.create_subscription(
            GoTo, "~/go_to", self._go_to, 10, callback_group=callback_group
        )

    @abstractmethod
    def set_group_mask(self, group_mask: float) -> None:
        """Gets called if a set_group_mask command is received from ros

        Args:
            group_mask (float): _description_
        """
        pass

    @abstractmethod
    def takeoff(
        self,
        group_mask: int,
        height: float,
        yaw: float,
        use_current_yaw: bool,
        duration_seconds: float,
    ) -> None:
        """Gets called if a takeoff command is received from ros

        Args:
            group_mask (int): The crazyflies addressed
            height (float): The target height (absolute) in meters
            yaw (float): The target yaw in rad
            use_current_yaw (bool): Wheater yaw is valid or current yaw should be maintained
            duration_seconds (float): The time it should take to reach the height
        """
        pass

    @abstractmethod
    def land(
        self,
        group_mask: int,
        height: float,
        yaw: float,
        use_current_yaw: bool,
        duration_seconds: float,
    ) -> None:
        """Gets called if a land command is received from ros

        Args:
            group_mask (int): _description_
            height (float): _description_
            yaw (float): _description_
            use_current_yaw (bool): _description_
            duration_seconds (float): _description_
        """
        pass

    @abstractmethod
    def stop(self, group_mask: int) -> None:
        """Gets called if a stop command is received from ros

        Stops the motors

        Args:
            group_mask (int): _description_
        """
        pass

    @abstractmethod
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
        """Gets called when a go_to command is received in ros

        Args:
            group_mask (int): _description_
            relative (bool): _description_
            linear (bool): _description_
            x (float): _description_
            y (float): _description_
            z (float): _description_
            yaw (float): _description_
            duration_seconds (float): _description_
        """

    def _set_group_mask(self, msg: SetGroupMask):
        self.set_group_mask(msg.group_mask)

    def _takeoff(self, msg: Takeoff):
        duration_seconds = msg.duration.sec + msg.duration.nanosec * 1e-9
        self.takeoff(
            msg.group_mask, msg.height, msg.yaw, msg.use_current_yaw, duration_seconds
        )

    def _land(self, msg: Land):
        duration_seconds = msg.duration.sec + msg.duration.nanosec * 1e-9
        self.land(
            msg.group_mask, msg.height, msg.yaw, msg.use_current_yaw, duration_seconds
        )

    def _stop(self, msg: Stop):
        self.stop(msg.group_mask)

    def _go_to(self, msg: GoTo):
        duration_seconds = msg.duration.sec + msg.duration.nanosec * 1e-9
        self.go_to(
            msg.group_mask,
            msg.relative,
            msg.linear,
            msg.goal.x,
            msg.goal.y,
            msg.goal.z,
            msg.yaw,
            duration_seconds,
        )
