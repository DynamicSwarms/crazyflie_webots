import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions.execute_process import ExecuteProcess
import yaml


def create_command(robot_name, description_path):
    webots_controller_path = (
        get_package_share_directory("webots_ros2_driver") + "/scripts/webots-controller"
    )
    webots_ip = EnvironmentVariable("WEBOTS_IP", default_value="192.168.56.1").perform(
        LaunchContext()
    )

    return (
        webots_controller_path
        + " --robot-name="
        + robot_name
        + " --protocol=tcp --ip-address="
        + str(webots_ip)
        + " --port=1234 ros2 --ros-args -p robot_description:="
        + description_path
    )


def load_ids(ids_config_yaml: str):
    """Get crazyflies and wands ids.

    In order to start the controllers for the wands and crazyflies the ids need to be known.
    A yaml can therfore be passed describing which robots are present in the simulation.

    Args:
        ids_config_yaml (str): a path to the yaml config
    """
    file = open(ids_config_yaml, "r")
    yaml_data = yaml.safe_load(file)
    cf_ids = yaml_data["crazyflies"]
    wand_ids = yaml_data["wands"]
    return cf_ids, wand_ids


def create_controllers(context):
    """
    Create the crazyflie and wand controller nodes.
    """
    package_dir = get_package_share_directory("crazyflie_webots")
    robots_yaml = LaunchConfiguration("robots_yaml").perform(context)
    cf_ids, wand_ids = load_ids(robots_yaml)

    wand_description_path = os.path.join(package_dir, "resource", "wand.urdf")
    for id in wand_ids:
        name = "Wand0" + str(id) + "_ros_ctrl"
        wand = ExecuteProcess(
            name="Wand",
            cmd=[create_command(name, wand_description_path)],
            output="screen",
            shell=True,
            emulate_tty=True,
            additional_env={"WEBOTS_HOME": get_package_prefix("webots_ros2_driver")},
        )

        yield wand

    cf_description_path = os.path.join(package_dir, "resource", "cf.urdf")
    for id in cf_ids:
        name = "cf" + str(id) + "_ros_ctrl"
        crazyflie = ExecuteProcess(
            name="Crazyflie",
            cmd=[create_command(name, cf_description_path)],
            output="screen",
            shell=True,
            emulate_tty=True,
            additional_env={"WEBOTS_HOME": get_package_prefix("webots_ros2_driver")},
        )

        yield crazyflie


def generate_launch_description():
    default_robots_yaml = (
        get_package_share_directory("crazyflie_webots") + "/launch/webots_robots.yaml"
    )
    robots_yaml_launch_arg = DeclareLaunchArgument(
        name="robots_yaml",
        default_value=default_robots_yaml,
        description="Path to a .yaml file which specifies which specifies which robots "
        + "are present in simulation. That way the ros controller for this can be launched.",
    )

    return LaunchDescription(
        [robots_yaml_launch_arg, OpaqueFunction(function=create_controllers)]
    )
