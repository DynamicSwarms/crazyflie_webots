import asyncio
import os

import rclpy
from rclpy.node import Node

from crazyflie_webots_gateway_interfaces.srv import WebotsCrazyflie
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from ros2run.api import get_executable_path

from launch.substitutions import EnvironmentVariable
from launch import LaunchContext

from signal import SIGINT, SIGKILL


class Gateway(Node):
    def __init__(self):
        super().__init__(
            "crazyflie_webots_gateway",
            automatically_declare_parameters_from_overrides=True,
        )

        self.add_service = self.create_service(
            WebotsCrazyflie, "~/add_crazyflie", self.add_crazyflie
        )
        self.remove_service = self.create_service(
            WebotsCrazyflie, "~/remove_crazyflie", self.remove_crazyflie
        )

        self.cfs = {}

    def add_crazyflie(self, req, resp):
        self.get_logger().info("Got called to add crazyflie with ID: {}".format(req.id))
        initial_position = [
            req.initial_position.x,
            req.initial_position.y,
            req.initial_position.z,
        ]
        asyncio.ensure_future(self.create_cf(req.id))
        resp.success = True
        return resp

    def remove_crazyflie(self, req, resp):
        self.get_logger().info(
            "Got called to remove Crazyflie with ID: {}".format(req.id)
        )
        process = self.cfs[req.id]
        os.killpg(os.getpgid(process.pid), SIGINT)
        resp.success = True
        return resp

    async def create_cf(self, id):
        webots_controller_path = (
            get_package_share_directory("webots_ros2_driver")
            + "/scripts/webots-controller"
        )
        description_path = os.path.join(
            get_package_share_directory("crazyflie_webots"), "resource", "cf.urdf"
        )
        webots_ip = EnvironmentVariable(
            "WEBOTS_IP", default_value="192.168.56.1"
        ).perform(LaunchContext())
        webots_home = get_package_prefix("webots_ros2_driver")

        robot_name = "cf{}_ros_ctrl".format(id)

        cmd = (
            "WEBOTS_HOME={} ".format(webots_home)
            + webots_controller_path
            + " --robot-name="
            + robot_name
            + " --protocol=tcp --ip-address="
            + str(webots_ip)
            + " --port=1234 ros2 --ros-args -p robot_description:="
            + description_path
        )
        self.get_logger().info(cmd)
        self.cfs[id] = await asyncio.create_subprocess_shell(cmd, preexec_fn=os.setsid)
        await self.cfs[id].wait()


async def run_node():
    gateway = Gateway()
    while rclpy.ok():
        await asyncio.sleep(0.01)
        rclpy.spin_once(gateway, timeout_sec=0)
    rclpy.shutdown()


def main():
    rclpy.init()
    event_loop = asyncio.get_event_loop()
    asyncio.ensure_future(run_node())
    event_loop.run_forever()


if __name__ == "__main__":
    main()
