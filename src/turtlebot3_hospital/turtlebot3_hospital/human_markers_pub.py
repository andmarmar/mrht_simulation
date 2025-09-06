import rclpy
from rclpy.node import Node
from hunav_msgs.msg import Agents
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray

ROBOT_NAMES = ['tb1', 'tb2', 'tb3']

class HumanAndRobotMarkersPub(Node):
    def __init__(self):
        super().__init__('human_and_robot_markers_pub')
        self.human_pub = self.create_publisher(MarkerArray, '/humans_marker_array', 10)
        self.robot_pub = self.create_publisher(MarkerArray, '/robots_marker_array', 10)
        self.create_subscription(Agents, '/human_states', self.human_marker, 10)
        self.robot_poses = {name: None for name in ROBOT_NAMES}
        for i, name in enumerate(ROBOT_NAMES):
            topic = f'/{name}/amcl_pose'
            self.create_subscription(PoseWithCovarianceStamped, topic, self.robot_callback(name), 10)

    # Marcador humano con forma de ovoide

    def human_marker(self, msg):
        ma = MarkerArray()
        for agent in msg.agents:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "humans"
            marker.id = agent.id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = agent.position
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 1.7
            marker.color.r = 0.2
            marker.color.g = 0.7
            marker.color.b = 1.0
            marker.color.a = 0.9
            marker.lifetime.sec = 1
            ma.markers.append(marker)
        self.human_pub.publish(ma)

    # Auxiliar para crear callback de la posición robot

    def robot_callback(self, name):
        def callback(msg):
            self.robot_poses[name] = msg.pose.pose
            self.robot_marker()
        return callback

    # Marcador robot con forma de caja 

    def robot_marker(self):
        ma = MarkerArray()
        for i, (name, pose) in enumerate(self.robot_poses.items()):
            if pose is None:
                continue
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "robots"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            marker.color.r = 1.0 
            marker.color.g = 0.2 
            marker.color.b = 0.2
            marker.color.a = 0.9
            marker.lifetime.sec = 0
            ma.markers.append(marker)
        self.robot_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = HumanAndRobotMarkersPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()