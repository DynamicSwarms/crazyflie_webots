import rclpy
from geometry_msgs.msg import TransformStamped, Vector3
from tf2_ros import TransformBroadcaster

from tf_transformations import quaternion_about_axis
import re

from typing import List, Dict


class WebotsRosDriver:
    num: int = 0

    def init(self, webots_node, properties):
        self.num += 1
        self.wb_node = self.getNode(webots_node)
        rclpy.init(args=None)
        self.ros_node = rclpy.create_node(str(self.getName()))
        self.translation_field = self.wb_node.getField("translation")
        self.rotation_field = self.wb_node.getField("rotation")
        self.tf_broadcaster = TransformBroadcaster(self.ros_node)

        self.log("Node started for " + str(self.getName()))
        self.ros_node.create_timer(1 / 10.0, self.broadcast_position)

    def step(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)

    def getNode(self, node):
        __ros_robot = node.robot
        __super = __ros_robot.getSelf()
        return __super.getParentNode()

    def getName(self):
        field = self.wb_node.getField("name")
        return field.getSFString()

    def broadcast_positions(self):
        # These might not be connected yet!!
        self.log(self.num)
        return
        """
        children_field = self.wb_node.getParentNode().getField("children")
        canditates: List["wb_node"] = []
        names: Dict[int, str] = {}
        if children_field:
            for i in range(children_field.getCount()):
                child_node = children_field.getMFNode(i)
                type_name = child_node.getTypeName()
                if type_name == "Crazyflie":
                    name = child_node.getField("name").getSFString()
                    match = re.search(r"\d+$", name)
                    number = int(match.group()) if match else None
                    canditates.append(child_node)
                    names[number] = name
        if names[min(names.keys())] == self.getName():
            self.log(canditates)
        """

    def broadcast_position(self):
        pos = self.get_position()
        rot = self.get_rotation()
        self.tf_broadcaster.sendTransform(self.prepare_transform(pos, rot))

    def get_position(self):
        return self.translation_field.getSFVec3f()

    def get_rotation(self):
        return self.rotation_field.getSFRotation()

    def log(self, txt):
        self.ros_node.get_logger().info(str(txt))

    def prepare_transform(self, pos, rot):
        t = TransformStamped()
        t.header.stamp = self.ros_node.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = self.getName()

        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]

        quat = quaternion_about_axis(rot[3], rot[:3])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        return t
