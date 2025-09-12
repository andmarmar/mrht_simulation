import rclpy
from rclpy.node import Node
from hunav_msgs.msg import Agents
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from turtlebot3_hospital_msgs.msg import Task
import math

ROBOT_NAMES = ['tb1', 'tb2', 'tb3']

class HumanAndRobotMarkersPub(Node):
    def __init__(self):
        super().__init__('human_and_robot_markers_pub')
        self.human_pub = self.create_publisher(MarkerArray, '/humans_marker_array', 10)
        self.robot_pub = self.create_publisher(MarkerArray, '/robots_marker_array', 10)
        self.task_pub = self.create_publisher(MarkerArray, '/tasks_marker_array', 10)

        self.create_subscription(Agents, '/human_states', self.human_marker, 10)
        self.create_subscription(Task, '/new_task', self.task_callback, 10)

        self.robot_poses = {name: None for name in ROBOT_NAMES}
        self.tasks = {}     
        self.human_poses = {} 
        for name in ROBOT_NAMES:
            topic = f'/{name}/amcl_pose'
            self.create_subscription(
                PoseWithCovarianceStamped,
                topic,
                self.robot_callback(name),
                10
            )

    # Marcador humano con forma de ovoide

    def human_marker(self, msg):
        ma = MarkerArray()
        self.human_poses = {}
        for agent in msg.agents:
            hx = agent.position.position.x
            hy = agent.position.position.y
            self.human_poses[agent.id] = (hx, hy)
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
        self.check_task()

    # Auxiliar para crear callback de la posición robot

    def robot_callback(self, name):
        def callback(msg):
            self.robot_poses[name] = msg.pose.pose
            self.robot_marker()
            self.check_task()
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

    # Callback de tarea

    def task_callback(self, msg):
            self.tasks[msg.id] = (float(msg.x), float(msg.y))
            self.task_markers()

    # Marcador tarea con forma de cilindro

    def task_markers(self):
        ma = MarkerArray()
        for tid, (x, y) in self.tasks.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "tasks"
            try:
                marker.id = int(str(tid).replace('t', ''))
            except Exception:
                marker.id = hash(tid) % 10000
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.lifetime.sec = 0
            ma.markers.append(marker)
        self.task_pub.publish(ma)

    # Borrar marcador tarea

    def delete_task(self, tid):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "tasks"
        try:
            marker.id = int(str(tid).replace('t', ''))
        except Exception:
            marker.id = hash(tid) % 10000
        marker.action = Marker.DELETE
        ma = MarkerArray()
        ma.markers.append(marker)
        self.task_pub.publish(ma)

    # Comprobar si alguna tarea se ha completado

    def check_task(self):
        to_remove = []

        for name, pose in self.robot_poses.items():
            if pose is None:
                continue
            rx, ry = pose.position.x, pose.position.y
            for tid, (tx, ty) in self.tasks.items():
                dist = math.hypot(rx - tx, ry - ty)
                if dist < 0.6:
                    to_remove.append(tid)

        for hid, (hx, hy) in self.human_poses.items():
            for tid, (tx, ty) in self.tasks.items():
                dist = math.hypot(hx - tx, hy - ty)
                if dist < 0.6:
                    to_remove.append(tid)

        for tid in set(to_remove):
            self.tasks.pop(tid, None)
            self.delete_task(tid)


def main(args=None):
    rclpy.init(args=args)
    node = HumanAndRobotMarkersPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()