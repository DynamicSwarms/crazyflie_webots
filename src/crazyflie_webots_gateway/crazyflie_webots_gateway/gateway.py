import os
import asyncio
from asyncio.subprocess import Process
from signal import SIGINT
import time
from threading import Thread

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.substitutions import EnvironmentVariable
from launch import LaunchContext

import rclpy
from rclpy import Future
from rclpy.node import Node
from rclpy.client import Client

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import ChangeState, GetState


from crazyflie_webots_gateway_interfaces.srv import WebotsCrazyflie

from dataclasses import dataclass
from typing import Dict, Optional, Tuple


class GatewayError(Exception):
    pass


@dataclass
class CrazyflieInstance:
    process: Optional[Process] = None


class Gateway(Node):
    def __init__(self, event_loop):
        super().__init__(
            "crazyflie_webots_gateway",
            automatically_declare_parameters_from_overrides=True,
        )

        self._cf_event_loop = event_loop
        self._load_paths_and_directories()
        self._logger = self.get_logger()

        self._logger.info(
            "Started Webots-Gateway with "
            + "IP: {} and PORT: {}. ".format(self._webots_ip, self._webots_port)
            + "Change this configuration by setting eviromental variable"
            + "'WEBOTS_IP' or 'WEBOTS_PORT' respectively. "
            + "(e.g. 'export WEBTOS_IP=192.168.56.1')"
        )
        self.crazyflies: Dict[int, CrazyflieInstance] = {}
        self.crazyflie_callback_group = MutuallyExclusiveCallbackGroup()

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

    def remove_crazyflie(self, id: int) -> Tuple[bool, str]:
        self._logger.info("Removing crazyflie with ID: {}".format(id))
        if id in self.crazyflies.keys():
            process = self.crazyflies[id].process
            os.killpg(os.getpgid(process.pid), SIGINT)
            return True, "Success"

        msg = "Couldn't remove crazyflie with ID: {}; was not active. ".format(id)
        self._logger.info(msg)
        return False, msg

    def remove_all_crazyflies(self) -> None:
        for cf_key in list(self.crazyflies.keys()):
            _ = self.remove_crazyflie(cf_key)
            del self.crazyflies[cf_key]

    def add_crazyflie(self, id: int) -> bool:
        self._logger.info("Adding crazyflie with ID: {}".format(id))
        if id in self.crazyflies.keys():
            raise GatewayError("Cannot add Crazyflie, is already in Gateway!")
        else:
            wait_future = asyncio.run_coroutine_threadsafe(
                self._create_cf(id), loop=self._cf_event_loop
            )
            wait_future.add_done_callback(lambda fut: self._on_crazyflie_exit(fut, id))
            success = True
        return success

    def _on_crazyflie_exit(self, fut: asyncio.Future, id: int):
        """Callback invoked when a crazyflie subprocess exits."""
        self._logger.info(f"Crazyflie (id={id}) exited.")

        # Clean up the crazyflie entry from the dictionary
        if id in self.crazyflies.keys():
            del self.crazyflies[id]

    async def _create_cf(self, id: int):
        cmd = self._create_start_command(id)

        process = await asyncio.create_subprocess_exec(*cmd, preexec_fn=os.setsid)

        self.crazyflies[id] = CrazyflieInstance(process=process)
        await self.crazyflies[id].process.wait()

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
        try:
            resp.success = self.add_crazyflie(req.id)
        except (TimeoutError, Gateway) as ex:
            self._logger.info(str(ex))
            resp.success = False
        return resp

    def _remove_crazyflie_callback(
        self, req: WebotsCrazyflie.Request, resp: WebotsCrazyflie.Response
    ) -> WebotsCrazyflie.Response:
        resp.success = self.remove_crazyflie(req.id)
        return resp


async def run_node(cf_eventloop):
    rclpy.init()
    gateway = Gateway(cf_eventloop)
    try:
        while rclpy.ok():
            await asyncio.sleep(0.01)
            rclpy.spin_once(gateway, timeout_sec=0)
        rclpy.shutdown()
    except asyncio.CancelledError:
        gateway.remove_all_crazyflies()


def run_crazyflie_loop(event_loop: asyncio.AbstractEventLoop):
    asyncio.set_event_loop(event_loop)

    async def keep_alive():
        try:
            while True:
                await asyncio.sleep(1)  # Keep the loop alive
        except asyncio.CancelledError:
            pass

    alive_task = asyncio.ensure_future(keep_alive())
    # If there is no future in the loop it cannot be stopped...

    event_loop.run_forever()
    alive_task.cancel()
    for fut in asyncio.all_tasks(event_loop):
        event_loop.run_until_complete(fut)
    event_loop.close()


def run_gateway_loop(
    event_loop: asyncio.AbstractEventLoop,
    crazyflie_event_loop: asyncio.AbstractEventLoop,
):
    asyncio.set_event_loop(event_loop)
    node = asyncio.ensure_future(run_node(crazyflie_event_loop), loop=event_loop)
    event_loop.run_forever()
    node.cancel()
    event_loop.run_until_complete(node)
    event_loop.close()


def main():
    gateway_event_loop = asyncio.new_event_loop()
    crazyflie_event_loop = asyncio.new_event_loop()

    crazyflie_thread = Thread(target=run_crazyflie_loop, args=(crazyflie_event_loop,))
    gateway_thread = Thread(
        target=run_gateway_loop, args=(gateway_event_loop, crazyflie_event_loop)
    )
    crazyflie_thread.start()
    gateway_thread.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        crazyflie_event_loop.stop()
        gateway_event_loop.stop()

    crazyflie_thread.join()
    gateway_thread.join()
    exit()


if __name__ == "__main__":
    main()
