import os 
import pathlib
import launch 
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from webots_ros2_driver.webots_controller import WebotsController

from launch.actions.execute_process import ExecuteProcess

def generate_launch_description():
    package_dir = get_package_share_directory('crazyflie_server')
    
    nodes = []
    
    wand_description_path = os.path.join(package_dir,  'resource', 'wand.urdf')
    #nodes.append(WebotsController(
    #    robot_name = 'Wand0' + str(1) + '_ros_ctrl',
    #    parameters = [{'robot_description': wand_description_path,
    #                    'ip-address' : '192.168.56.1',
    #                    'protocol' : 'tcp'
    #                    },
    #                ],
    #    respawn = True
    #))
    
    proc = ExecuteProcess(
        name="Wand",
        cmd=[get_package_share_directory('webots_ros2_driver') + "/scripts/webots-controller --robot-name=Wand01_ros_ctrl --protocol=tcp --port=1234 --ip-address=192.168.56.1 --port=1234 ros2 --ros-args -p robot_description:=" + package_dir +  "/resource/wand.urdf"],
        output="screen",
        shell=True,
        emulate_tty=True,
        additional_env={'WEBOTS_HOME': get_package_prefix('webots_ros2_driver')}
        )
        
    
    nodes.append(proc)
    
    return LaunchDescription(nodes)
