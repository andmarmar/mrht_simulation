import rclpy
from rclpy.node import Node
import yaml
import random
import time

from turtlebot3_hospital_msgs.msg import Task


class TaskGenerator(Node):
    def __init__(self):
        super().__init__('task_generator')

        self.pub = self.create_publisher(Task, '/new_task', 10)

        self.declare_parameter('task_file', 'config/tasks.yaml')
        self.task_file = self.get_parameter('task_file').get_parameter_value().string_value

        self.load_tasks()

        self.get_logger().info(f"Cargadas {len(self.tasks)} tareas desde {self.task_file}")

        self.create_timer(1.0, self.task_loop)

        self.last_sent = time.time()

    # Carga de tareas desde tasks.yaml

    def load_tasks(self):
        with open(self.task_file, 'r') as f:
            data = yaml.safe_load(f)
        self.tasks = data['tasks']
        random.shuffle(self.tasks)

    # Bucle principal de envio de tareas

    def task_loop(self):
        if not self.tasks:
            self.load_tasks()
            return

        now = time.time()

        if now - self.last_sent > random.uniform(3, 8):
            t = self.tasks.pop()
            msg = Task()
            msg.id = t['id']
            msg.x = float(t['x'])
            msg.y = float(t['y'])
            msg.frame_id = t.get('frame_id', 'map')

            msg.duration = random.uniform(10, 30)

            msg.exclusive = random.choices([0, 1, 2], weights=[85, 15, 0], k=1)[0]

            self.pub.publish(msg)
            self.last_sent = now
            self.get_logger().info(
                f"Nueva tarea publicada: {msg.id} (duration={msg.duration:.1f}s, exclusive={msg.exclusive})")


def main(args=None):
    rclpy.init(args=args)
    node = TaskGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
