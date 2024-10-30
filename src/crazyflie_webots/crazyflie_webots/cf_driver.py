from crazyflie_interfaces.msg._set_group_mask import SetGroupMask
import rclpy
from crazyflie_webots.wb_ros_driver import WebotsRosDriver
from crazyflie_interfaces.msg import Position, Land, Takeoff, GoTo, GenericLogData

from crazyflie_driver.high_level_commander import WebotsHighLevelCommander
from crazyflie_driver.generic_commander import WebotsGenericCommander
from crazyflie_driver.rpyt_commander import WebotsRPYTCommander
from crazyflie_driver.logging import WebotsLogging
from crazyflie_driver.parameters import WebotsParameters


from typing import List


class CrazyflieDriverNode(WebotsRosDriver):
    def init(self, webots_node, properties):
        super().init(webots_node, properties)
        self.target_field = self.wb_node.getField("target")
        self.range_finder = self.wb_node.getField("zrange")

        name = str(self.getName())

        hl_commander = WebotsHighLevelCommander(
            self.ros_node, self.set_target, self.get_position
        )

        generic_commander = WebotsGenericCommander(
            self.ros_node, self.set_target, self.get_position
        )

        rpyt_commander = WebotsRPYTCommander(self.ros_node)

        logging_variables = {"range.zrange": self.get_zrange}
        logging = WebotsLogging(self.ros_node, logging_variables)

        parameters = WebotsParameters(self.ros_node)

        logging.add_block(0, [{"group": "range", "name": "zrange"}])
        logging.start_block(0, 50, "zrange")

        # self.zrange_timer = self.ros_node.create_timer(
        #    1.0 / 20.0, self.z_range_timer_callback
        # )

    # def z_range_timer_callback(self):
    #    msg = GenericLogData()
    #    msg.values.append(self.range_finder.getSFFloat())
    #    self.zrange_pub.publish(msg)
    #
    def get_zrange(self) -> float:
        return self.range_finder.getSFFloat()

    def set_target(self, target: List[float]) -> None:
        self.target_field.setSFVec3f(target)

    def step(self):
        super().step()
