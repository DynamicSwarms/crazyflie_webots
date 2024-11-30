from crazyflie_webots.wb_ros_driver import WebotsRosDriver

from crazyflie_driver.high_level_commander import WebotsHighLevelCommander
from crazyflie_driver.generic_commander import WebotsGenericCommander
from crazyflie_driver.rpyt_commander import WebotsRPYTCommander
from crazyflie_driver.logging import WebotsLogging
from crazyflie_driver.parameters import WebotsParameters

from crazyflie_interfaces_python.server.logblock import LogBlockServer
from typing import List


class CrazyflieDriverNode(WebotsRosDriver):
    def init(self, webots_node, properties):
        super().init(webots_node, properties)
        self.target_field = self.wb_node.getField("target")
        self.range_finder = self.wb_node.getField("zrange")
        self.vbat_field = self.wb_node.getField("vbat")
        self.charge_current_field = self.wb_node.getField("chargeCurrent")
        self.pm_state_field = self.wb_node.getField("pm_state")

        hl_commander = WebotsHighLevelCommander(
            self.ros_node, self.set_target, self.get_position
        )

        generic_commander = WebotsGenericCommander(
            self.ros_node, self.set_target, self.get_position
        )

        rpyt_commander = WebotsRPYTCommander(self.ros_node)

        parameters = WebotsParameters(self.ros_node)

        logging_variables = {
            "range.zrange": self.get_zrange,
            "stateEstimate.x": lambda: self.get_position()[0],
            "stateEstimate.y": lambda: self.get_position()[1],
            "stateEstimate.z": lambda: self.get_position()[2],
        }
        if self.vbat_field is not None:
            logging_variables["pm.vbat"] = self.get_vbat
        if self.charge_current_field is not None:
            logging_variables["pm.chargeCurrent"] = self.get_charge_current
        if self.pm_state_field is not None:
            logging_variables["pm.state"] = self.get_pm_state

        logging = WebotsLogging(self.ros_node, logging_variables)

        # block = LogBlockServer(self.ros_node, "zrange")
        # logging.create_log_block(["range.zrange"], block)
        # block._log_block_start_callback(200)

    def get_zrange(self) -> float:
        return self.range_finder.getSFFloat()

    def set_target(self, target: List[float]) -> None:
        self.target_field.setSFVec3f(target)

    def get_vbat(self) -> float:
        if self.vbat_field is not None:
            return self.vbat_field.getSFFloat()
        return 0.0

    def get_charge_current(self) -> float:
        if self.charge_current_field is not None:
            return self.charge_current_field.getSFFloat()
        return 0.0

    def get_pm_state(self) -> float:
        if self.pm_state_field is not None:
            return float(self.pm_state_field.getSFInt32())
        return 0.0

    def step(self):
        super().step()
