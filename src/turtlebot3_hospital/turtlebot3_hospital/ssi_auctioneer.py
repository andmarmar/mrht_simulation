import math
import json
from typing import Dict, List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from hunav_msgs.msg import Agents
from turtlebot3_hospital_msgs.msg import Task as TaskMsg


from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# Estructuras 

class Task:
    def __init__(self, task_id: str, x: float, y: float, frame_id: str = "map",
                duration: float = 0, exclusive: int = 0):
        self.id = task_id
        self.x = x
        self.y = y
        self.frame_id = frame_id
        self.duration = duration
        self.exclusive = exclusive 

    def pose_stamped(self) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.orientation.w = 1.0
        return msg

class AgentState:
    def __init__(self, name: str, a_type: str, speed: float):
        self.name = name              
        self.type = a_type            
        self.speed = speed            
        self.pose_xy: Optional[Tuple[float, float]] = None
        self.rest_pose: Optional[Tuple[float, float]] = None
        self.queue: List[Task] = []   
        self.wait_until: Optional[float] = None
        self.busy: bool = False
        self.current_task: Optional[Task] = None

# Nodo principal 

class SSIAuctioneer(Node):
    def __init__(self):
        super().__init__('ssi_auctioneer')

        
        self.declare_parameter('agents', r'''
        [
          {"name":"tb1","type":"robot","speed":0.22},
          {"name":"tb2","type":"robot","speed":0.22},
          {"name":"tb3","type":"robot","speed":0.22},
          {"name":"human_1","type":"human","speed":1.20},
          {"name":"human_2","type":"human","speed":1.20},
          {"name":"human_3","type":"human","speed":1.20},
          {"name":"human_4","type":"human","speed":1.20}
        ]
        ''')

        self.declare_parameter('fast_dispatch', True) 
        self.declare_parameter('amcl_topic_suffix', 'amcl_pose') 
        self.declare_parameter('human_states_topic', '/human_states')

        self.fast_dispatch = self.get_parameter('fast_dispatch').get_parameter_value().bool_value
        self.amcl_suffix = self.get_parameter('amcl_topic_suffix').get_parameter_value().string_value
        self.human_states_topic = self.get_parameter('human_states_topic').get_parameter_value().string_value

        agents_param = self.get_parameter('agents').get_parameter_value().string_value
        self.agents: Dict[str, AgentState] = {}
        try:
            for a in json.loads(agents_param):
                self.agents[a['name']] = AgentState(a['name'], a['type'], float(a['speed']))
        except Exception as e:
            self.get_logger().error(f"Error parsing 'agents' param: {e}")
            raise

        self.tasks: List[Task] = []

        # ActionClients para robots y publishers para humanos
        
        self.robot_action_clients: Dict[str, ActionClient] = {}
        self.human_goal_publishers: Dict[str, rclpy.node.Publisher] = {}

        self.create_subscription(TaskMsg, '/new_task', self.task_callback, 10)


        for name, a in self.agents.items():
            if a.type == 'robot':
                ns = f'/{name}'
                action_name = f'{ns}/navigate_to_pose'
                self.robot_action_clients[name] = ActionClient(self, NavigateToPose, action_name)

                
                qos_amcl = QoSProfile(
                    depth=1,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
                )

                self.create_subscription(
                    PoseWithCovarianceStamped,
                    f'{ns}/{self.amcl_suffix}',
                    lambda msg, who=name: self.robot_callback(who, msg),
                    qos_amcl
                )
            else:
                hid = name.split('_')[-1]
                topic = f'/human_{hid}/set_goal'
                self.human_goal_publishers[name] = self.create_publisher(PoseStamped, topic, 10)
        
        # Subscripcion para posiciones de humanos
        self.create_subscription(Agents, self.human_states_topic, self.human_callback, 10)

        # Lanza el bucle principal cada 0.5 s

        self.timer = self.create_timer(0.5, self.auction_loop)

        self.robot_goal_handles = {}

        self.get_logger().info('Planificador de tareas activo')

    # Callbakcs 

    def robot_callback(self, name: str, msg: PoseWithCovarianceStamped):
        self.agents[name].pose_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.agents[name].rest_pose is None:
            self.agents[name].rest_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def human_callback(self, msg: Agents):
        id_to_name = {int(n.split('_')[-1]): n for n in self.agents if self.agents[n].type=='human'}
        for a in msg.agents:
            nm = id_to_name.get(a.id)
            if nm is not None:
                self.agents[nm].pose_xy = (a.position.position.x, a.position.position.y)
                if self.agents[nm].rest_pose is None:
                    self.agents[nm].rest_pose = (a.position.position.x, a.position.position.y)
            

    # Bucle principal

    def auction_loop(self):

        for name, a in self.agents.items():
            if a.type == 'human' and a.pose_xy and a.current_task:
                task = a.current_task
                dist = math.hypot(task.x - a.pose_xy[0], task.y - a.pose_xy[1])

                if dist < 1 and a.wait_until is None:
                    self.get_logger().info(f"Humano {name} llegó a {task.id}, iniciando espera {task.duration}s")
                    a.wait_until = self.get_clock().now().nanoseconds * 1e-9 + task.duration

            if a.wait_until is not None:
                now = self.get_clock().now().nanoseconds * 1e-9
                if now >= a.wait_until:
                    a.wait_until = None
                    self.after_wait(name)


        if not all(v.pose_xy is not None for v in self.agents.values()):
            return

        while self.tasks:
            t = self.tasks[0]
            winner, winner_cost = self.start_auction(t)
            if winner is None:
                break
            self.assign_task(self.agents[winner], t)
            self.tasks.pop(0)

    # Subasta de tareas
    def start_auction(self, task: Task) -> Tuple[Optional[str], float]:
        best_name = None
        best_cost = math.inf
        cost_details = {}
        for name, a in self.agents.items():
            if task.exclusive == 1 and a.type != 'robot':
                continue
            if task.exclusive == 2 and a.type != 'human':
                continue

            c, details = self.cost_estimate(a, task, return_details=True)
            cost_details[name] = details
            if c < best_cost:
                best_cost = c
                best_name = name
        
        # Información de costes en cada tarea
        for name, det in cost_details.items():
            self.get_logger().info(
            f" Coste  {name}: total={det['total']:.1f}s "
            f"(viaje={det['travel']:.1f}s, actual={det['current']:.1f}s, cola={det['queue']:.1f}s, duracion={det['task']:.1f}s)"
        )
        if best_name is not None:
            self.get_logger().info(f"Tarea {task.id} asignada a {best_name} (coste {best_cost:.2f}s)")
        return best_name, best_cost

    # Calculo de costes 

    def cost_estimate(self, a: AgentState, task: Task, return_details=False) -> float:
        if a.pose_xy is None:
            if return_details:
                return math.inf, {"current":0,"queue":0,"travel":0,"task":task.duration,"total":math.inf}
            return math.inf

        now = self.get_clock().now().nanoseconds * 1e-9
        current_cost = 0.0
        if a.current_task:
            current_cost = max(0.0, a.wait_until - now) if a.wait_until else a.current_task.duration

        queue_cost = sum(t.duration for t in a.queue)

        if a.queue:
            start_xy = (a.queue[-1].x, a.queue[-1].y)
        elif a.current_task:
            start_xy = (a.current_task.x, a.current_task.y)
        else:
            start_xy = a.pose_xy

        dist = math.hypot(task.x - start_xy[0], task.y - start_xy[1])
        travel_cost = dist / max(a.speed, 0.05)

        total_cost = current_cost + queue_cost + travel_cost + task.duration

        if return_details:
            return total_cost, {
                "current": current_cost,
                "queue": queue_cost,
                "travel": travel_cost,
                "task": task.duration,
                "total": total_cost
            }

        return total_cost


    # Asignar tarea

    def assign_task(self, a: AgentState, task: Task):
        a.queue.append(task)
        if self.fast_dispatch and not a.busy:
            self.dispatch_next(a)

    # Despachar tarea

    def dispatch_next(self, a: AgentState):
        if not a.queue:
            if a.rest_pose:
                rx, ry = a.rest_pose
                if a.pose_xy:
                    dist = math.hypot(a.pose_xy[0] - rx, a.pose_xy[1] - ry)
                else:
                    dist = float('inf') 
                if dist < 0.7 or a.busy or a.wait_until is not None:
                    return
                rest_task = Task(f"rest_{a.name}", a.rest_pose[0], a.rest_pose[1], frame_id="map")
                self.get_logger().info(f"{a.name} sin tareas, enviando a reposo en {a.rest_pose}")
                if a.type == 'robot':
                    self.send_r_goal(a.name, rest_task)
                else:
                    self.send_h_goal(a.name, rest_task)
                a.busy = True
            else:
                a.busy = False
            return
        next_task = a.queue.pop(0)
        a.current_task = next_task
        if a.type == 'robot':
            self.send_r_goal(a.name, next_task)
        else:
            self.send_h_goal(a.name, next_task)
        a.busy = True

    # Enviar goal a robot o humano

    def send_h_goal(self, name: str, task: Task):
        pub = self.human_goal_publishers[name]
        goal = task.pose_stamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        pub.publish(goal)

        a = self.agents[name]
        a.current_task = task
        a.busy = True

        self.get_logger().info(f"Human {name}: goal publicado -> ({task.x:.2f}, {task.y:.2f})")


    def send_r_goal(self, name: str, task: Task):
        ac = self.robot_action_clients[name]
        if not ac.server_is_ready():
            self.get_logger().warn(f"Nav2 no listo para {name}, reintenta en el siguiente tick ")
            self.agents[name].queue.insert(0, task)
            self.agents[name].busy = False
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = task.pose_stamped()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        send_future = ac.send_goal_async(goal_msg, feedback_callback=lambda fb, who=name: None)
        def goal_accepted(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f"Goal {name} rechadazo por Nav2")
                self.goal_callback(name)  
                return
            self.robot_goal_handles[name] = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda _: self.goal_callback(name))
        send_future.add_done_callback(goal_accepted)

    # Callback al llegar a la meta robot

    def goal_callback(self, name: str):
        a = self.agents[name]
        if not hasattr(a, 'current_task'):
            a.busy = False
            self.dispatch_next(a)
            return

        finished_task = a.current_task
        a.current_task = None 


        if finished_task is None:
            a.busy = False
            self.dispatch_next(a)
            return

        if finished_task.duration > 0:
            self.get_logger().info(
                f"{name}: llegó a {finished_task.id}, esperando {finished_task.duration:.1f}s"
            )
            a.wait_until = self.get_clock().now().nanoseconds * 1e-9 + finished_task.duration
        else:
            a.busy = False
            self.get_logger().info(f"{name}: goal completado. Despachando siguiente si existe")
            self.dispatch_next(a)

    # Acciones tras espera de humanos

    def after_wait(self, name: str):
        a = self.agents[name]
        a.busy = False
        a.current_task = None 
        self.get_logger().info(f"{name}: espera terminada. Despachando siguiente")
        self.dispatch_next(a)

    # Callback nueva tarea

    def task_callback(self, msg: TaskMsg):
        task = Task(msg.id, msg.x, msg.y, msg.frame_id, msg.duration, msg.exclusive)
        self.tasks.append(task)
        self.get_logger().info(f"Nueva tarea recibida: {task.id}")


        


def main(args=None):
    rclpy.init(args=args)
    node = SSIAuctioneer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()