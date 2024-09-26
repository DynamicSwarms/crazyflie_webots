import rclpy
from geometry_msgs.msg import TransformStamped, Vector3
from tf2_ros import TransformBroadcaster

from tf_transformations import quaternion_about_axis

class WebotsRosDriver:
    def init(self, webots_node, properties):
        self.wb_node = self.getNode(webots_node)
        rclpy.init(args=None)
        self.ros_node = rclpy.create_node(str(self.getName()) + '_driver')

        self.translation_field = self.wb_node.getField('translation')
        self.rotation_field = self.wb_node.getField('rotation')
        self.tf_broadcaster = TransformBroadcaster(self.ros_node)

        self.log("Node stared for " + str(self.getName()))

    def step(self):
        rclpy.spin_once(self.ros_node, timeout_sec = 0)
        self.broadcast_position()


    def getNode(self, node):
        __ros_robot = node.robot
        __super = __ros_robot.getSelf()
        return __super.getParentNode()
    
    def getName(self):
        field = self.wb_node.getField('name')
        return field.getSFString()
    
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
        t.header.frame_id = 'world'
        t.child_frame_id = self.getName()

        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]

        quat = quaternion_about_axis(rot[3] , rot[:3])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        return t