import asyncio
import os

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crazyflie_webots_gateway_interfaces.srv import WebotsCrazyflie
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch.substitutions import EnvironmentVariable
from launch import LaunchContext

from signal import SIGINT
from typing import Dict

from asyncio.subprocess import Process


class Gateway(Node):
    def __init__(self):
        super().__init__(
            "crazyflie_webots_gateway",
            automatically_declare_parameters_from_overrides=True,
        )
        self._load_paths_and_directories()
        self._logger = self.get_logger()

        self._logger.info(
            "Started Webots-Gateway with "
            + "IP: {} and PORT: {}. ".format(self._webots_ip, self._webots_port)
            + "Change this configuration by setting eviromental variable"
            + "'WEBOTS_IP' or 'WEBOTS_PORT' respectively. "
            + "(e.g. 'export WEBTOS_IP=192.168.56.1')"
        )

        self.crazyflies: Dict[int, Process] = {}

        callback_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            WebotsCrazyflie,
            "~/add_crazyflie",
            self._add_crazyflie_callback,
            callback_group=callback_group,
        )

        self.create_service(
            WebotsCrazyflie,
            "~/remove_crazyflie",
            self._remove_crazyflie_callback,
            callback_group=callback_group,
        )

    def remove_crazyflie(self, id: int) -> bool:
        self._logger.info("Removing crazyflie with ID: {}".format(id))
        if id in self.crazyflies.keys():
            process = self.crazyflies[id]
            os.killpg(os.getpgid(process.pid), SIGINT)
            return True

        self._logger.info(
            "Couldn't remove crazyflie with ID: {}; was not active. ".format(id)
        )
        return False

    def remove_all_crazyflies(self) -> None:
        for cf_key in self.crazyflies.keys():
            _ = self.remove_crazyflie(cf_key)

    def add_crazyflie(self, id: int) -> bool:
        self._logger.info("Adding crazyflie with ID: {}".format(id))
        asyncio.ensure_future(self._create_cf(id))
        return True

    async def _create_cf(self, id: int):
        cmd = self._create_start_command(id)
        self.crazyflies[id] = await asyncio.create_subprocess_exec(
            *cmd, preexec_fn=os.setsid
        )
        await self.crazyflies[id].wait()

    def _create_start_command(self, id: int) -> str:
        robot_name = "cf{}_ros_ctrl".format(id)

        return [
            "env",
            "WEBOTS_HOME={}".format(self._webots_home),
            self._webots_controller_path,
            "--robot-name={}".format(robot_name),
            "--protocol=tcp",
            "--ip-address={}".format(self._webots_ip),
            "--port={}".format(self._webots_port),
            "ros2",
            "--ros-args",
            "-p",
            "robot_description:={}".format(self._cf_description_path),
        ]

    def _load_paths_and_directories(self):
        self._webots_home = get_package_prefix("webots_ros2_driver")

        self._webots_controller_path = (
            get_package_share_directory("webots_ros2_driver")
            + "/scripts/webots-controller"
        )

        self._webots_ip = EnvironmentVariable(
            "WEBOTS_IP", default_value="127.0.0.1"
        ).perform(LaunchContext())

        self._webots_port = EnvironmentVariable(
            "WEBOTS_PORT", default_value="1234"
        ).perform(LaunchContext())

        self._cf_description_path = os.path.join(
            get_package_share_directory("crazyflie_webots"), "resource", "cf.urdf"
        )

    def _add_crazyflie_callback(
        self, req: WebotsCrazyflie.Request, resp: WebotsCrazyflie.Response
    ) -> WebotsCrazyflie.Response:
        resp.success = self.add_crazyflie(req.id)
        return resp

    def _remove_crazyflie_callback(
        self, req: WebotsCrazyflie.Request, resp: WebotsCrazyflie.Response
    ) -> WebotsCrazyflie.Response:
        resp.success = self.remove_crazyflie(req.id)
        return resp


async def run_node():
    rclpy.init()
    gateway = Gateway()
    try:
        while rclpy.ok():
            await asyncio.sleep(0.01)
            rclpy.spin_once(gateway, timeout_sec=0)
        rclpy.shutdown()
    except asyncio.CancelledError:
        gateway.remove_all_crazyflies()


def main():
    event_loop = asyncio.get_event_loop()
    node = asyncio.ensure_future(run_node())
    try:
        event_loop.run_forever()
    except KeyboardInterrupt:
        node.cancel()
        event_loop.run_until_complete(node)
    finally:
        event_loop.close()


if __name__ == "__main__":
    main()
