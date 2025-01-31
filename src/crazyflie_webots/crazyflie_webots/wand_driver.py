from crazyflie_webots.wb_ros_driver import WebotsRosDriver
from geometry_msgs.msg import Twist


class WandDriverNode(WebotsRosDriver):
    def init(self, webots_node, properties):
        super().init(webots_node, properties)

        self.ros_node.create_subscription(Twist, "cmd_vel", self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist
        x = twist.linear.x
        y = twist.linear.y
        z = twist.linear.z
        # self.target_field.setSFVec3f([x, y, z])

    def step(self):
        super().step()

    # Overwrite because this is weird
    def getName(self):
        field = self.wb_node.getField("wand_id")
        id = field.getSFInt32()
        return "Wand" + str(id)
