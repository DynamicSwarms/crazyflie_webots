import rclpy
from crazyflie_server.wb_ros_driver import WebotsRosDriver

from swarm_interface.msg import Position, Land, Takeoff, GoTo


class CrazyflieDriverNode(WebotsRosDriver):
    def init(self, webots_node, properties):
        super().init(webots_node, properties)
        self.target_field = self.wb_node.getField('target')
        

        name = str(self.getName())
        self.ros_node.create_subscription(Position, name + '/cmd_position', self.cmd_position, 1)
        self.ros_node.create_subscription(Land, name + '/land', self.land, 1)
        self.ros_node.create_subscription(Takeoff, name + '/takeoff', self.takeoff, 1)
        self.ros_node.create_subscription(GoTo, name + '/go_to', self.go_to, 1)
    
    def cmd_position(self, position):
        self.set_target([position.x, position.y, position.z])

    def land(self, land):
        pos = self.get_position()
        self.set_target([pos[0], pos[1], land.height + 0.02])

    def takeoff(self, takeoff):
        pos = self.get_position()
        self.set_target([pos[0], pos[1], takeoff.height])

    def go_to(self, goto):
        self.set_target([goto.goal.x, goto.goal.y, goto.goal.z])
    
    def set_target(self, target):
        self.target_field.setSFVec3f(target)

    def step(self):
        super().step()
        