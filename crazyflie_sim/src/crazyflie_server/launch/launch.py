import os 
from launch import LaunchDescription, LaunchContext
from launch.substitutions import EnvironmentVariable
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions.execute_process import ExecuteProcess

def create_command(robot_name, description_path):
    webots_controller_path = get_package_share_directory('webots_ros2_driver') + "/scripts/webots-controller" 
    webots_ip = EnvironmentVariable('WEBOTS_IP', default_value="192.168.56.1").perform(LaunchContext())

    return webots_controller_path + " --robot-name=" + robot_name + " --protocol=tcp --ip-address=" + str(webots_ip) + " --port=1234 ros2 --ros-args -p robot_description:=" + description_path

def generate_launch_description():
    package_dir = get_package_share_directory('crazyflie_server')
    
    nodes = []
    
    cf_ids = [0]
    wand_ids = [1]

    wand_description_path = os.path.join(package_dir,  'resource', 'wand.urdf')
    for id in wand_ids: 
        name = "Wand0" + str(id) + "_ros_ctrl"
        wand = ExecuteProcess(
            name="Wand",
            cmd=[create_command(name, wand_description_path)],
            output="screen",
            shell=True,
            emulate_tty=True,
            additional_env={'WEBOTS_HOME': get_package_prefix('webots_ros2_driver')}
            )
        
        nodes.append(wand)


    cf_description_path = os.path.join(package_dir,  'resource', 'cf.urdf')
    for id in cf_ids: 
        name = "cf" + str(id) + "_ros_ctrl"
        crazyflie = ExecuteProcess(
            name="Crazyflie",
            cmd=[create_command(name, cf_description_path)],
            output="screen",
            shell=True,
            emulate_tty=True,
            additional_env={'WEBOTS_HOME': get_package_prefix('webots_ros2_driver')}
        )
        
        nodes.append(crazyflie)
        
    return LaunchDescription(nodes)
