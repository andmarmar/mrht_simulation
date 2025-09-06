#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from hunav_msgs.msg import AgentGoal, Agents 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq
import math
import cv2

class HumanPlanner(Node):
    def __init__(self):
        super().__init__('human_planner')

        self.declare_parameter("human_id", 1)
        self.human_id = self.get_parameter("human_id").get_parameter_value().integer_value

        self.map_data = None
        self.current_pose = None
        self.goal_pose = None
        self.path = []
        self.current_waypoint = None
        self.waypoint_sent_time = None
        self.replan_timeout = 8.0 
        self.prev_waypoint = None

        self.get_logger().info(f'Planificador de trayectorias humano inicializado para human_id={self.human_id}')

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscripciones

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(Agents, '/human_states', self.human_states_callback, 10)
        self.create_subscription(PoseStamped, f'/human_{self.human_id}/set_goal', self.goal_callback, 10)

        # Publishers

        self.goal_pub = self.create_publisher(AgentGoal, '/hunav/set_agent_goal', 10)
        self.path_pub = self.create_publisher(Path, f'/human_{self.human_id}/path', 10)

        # Temporizador para bucle waypoints

        self.create_timer(0.05, self.send_waypoint)

    # Callbacks

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info('Recibido nuevo mapa')
        self.map_data = msg
        if self.current_pose and self.goal_pose and not self.path:
            self.get_logger().info('Tengo mapa y posicion, planifica ruta')
            self.set_goal(*self.goal_pose)

    def human_states_callback(self, msg: Agents):
        found = False
        for agent in msg.agents:
            if agent.id == self.human_id:
                self.current_pose = agent.position
                found = True
                if self.map_data and self.goal_pose and not self.path:
                    self.get_logger().info('Tengo mapa y posicion, planifica ruta')
                    self.set_goal(*self.goal_pose)
                break
        if not found:
            self.get_logger().warn(f'No se ha encontrado el agente con id={self.human_id} en /human_states')

    def goal_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f'Meta recibida desde tópico: x={x}, y={y}')
        self.set_goal(x, y)

    # Planificacion de camino
    def set_goal(self, x, y):
        self.goal_pose = (x, y)
        if not self.map_data or not self.current_pose:
            self.get_logger().warn("No se puede planificar aún (falta mapa o pose)")
            return

        start = self.get_map_index(self.current_pose)
        goal = self.map_coords(x, y)
        self.get_logger().info(f'Planificando camino desde {start} hasta {goal}')
        self.path = self.find_path(start, goal)
        ## 
        #raw_path = self.find_path(start, goal)
        #self.path = self.downsample_path(raw_path, step=1)
        ##
        self.get_logger().info(f'Camino planificado con {len(self.path)} waypoints')
        self.publish_path(self.path)

    # Algorimo A* 

    def find_path(self, start, goal):
        map_w = self.map_data.info.width
        map_h = self.map_data.info.height
        resolution = self.map_data.info.resolution

        data = np.array(self.map_data.data).reshape((map_h, map_w))
        data = np.where(data > 10, 1, 0)

        # Secciones de mapa que se han rellenado como ocuapdas
        # Vertices en coordenadas de mapa
        v1 = self.map_coords(-3.4, 2.83)
        v2 = self.map_coords(-3.4, 6.51)
        v3 = self.map_coords(0.0, 6.51)

        # Comprueba si un punto está dentro del triangulo
        def point_in_triangle(px, py, v1, v2, v3):
            def sign(p1, p2, p3):
                return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])
            b1 = sign((px, py), v1, v2) < 0.0
            b2 = sign((px, py), v2, v3) < 0.0
            b3 = sign((px, py), v3, v1) < 0.0
            return ((b1 == b2) and (b2 == b3))

        # Marca como ocupadas las celdas dentro del triangulo
        for y in range(map_h):
            for x in range(map_w):
                if point_in_triangle(x, y, v1, v2, v3):
                    data[y, x] = 1 

        margin_m = 0.15
        inflation_cells = math.ceil(margin_m / resolution)
        kernel = np.ones((inflation_cells*2+1, inflation_cells*2+1), np.uint8)
        inflated = cv2.dilate(data.astype(np.uint8), kernel, iterations=1)

        free_mask = (inflated == 0).astype(np.uint8)
        dist_transform = cv2.distanceTransform(free_mask, cv2.DIST_L2, 5)
        dist_norm = cv2.normalize(dist_transform, None, 0, 1.0, cv2.NORM_MINMAX)

        # Coste basado en distancia a obstaculos

        def cell_cost(x, y):
            d = dist_norm[y, x]
            penalty = (1.0 - d) ** 2 * 60.0
            return 1.0 + penalty

        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start, [start]))
        visited = set()

        while open_set:
            f, g, current, path = heapq.heappop(open_set)
            if current == goal:
                return [self.grid_to_world(p) for p in path]
            if current in visited:
                continue
            visited.add(current)

            x, y = current
            neighbors = [(x+dx, y+dy) for dx,dy in [(-1,0),(1,0),(0,-1),(0,1)]]
            for nx, ny in neighbors:
                if 0 <= nx < map_w and 0 <= ny < map_h and inflated[ny,nx] == 0:
                    if (nx, ny) not in visited:
                        extra_cost = cell_cost(nx, ny)
                        new_g = g + extra_cost
                        heapq.heappush(open_set, (new_g + self.heuristic((nx,ny), goal),
                                              new_g, (nx,ny), path+[(nx,ny)]))

        self.get_logger().warn('No se ha encontrado camino hasta la meta')
        return []

    def heuristic(self, a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    # Transformaciones de coordenadas
    def get_map_index(self, pose):
        mx = int((pose.position.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        my = int((pose.position.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        return (mx, my)

    def map_coords(self, x, y):
        mx = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        my = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        return (mx, my)

    def grid_to_world(self, cell):
        x = cell[0]*self.map_data.info.resolution + self.map_data.info.origin.position.x + self.map_data.info.resolution/2
        y = cell[1]*self.map_data.info.resolution + self.map_data.info.origin.position.y + self.map_data.info.resolution/2
        return (x, y)

    # Monitoreo de waypoints
    def send_waypoint(self):
        if not self.path and self.current_waypoint is None:
            return

        now = self.get_clock().now().seconds_nanoseconds()[0] + \
              self.get_clock().now().seconds_nanoseconds()[1] * 1e-9

        # Si no hay waypoint actual, envía el siguiente y guarda el tiempo
        if self.current_waypoint is None and self.path:
            self.current_waypoint = self.path.pop(0)
            self.publish_waypoint(self.current_waypoint)
            self.waypoint_sent_time = now
            self.prev_waypoint = None
            return

        # Si hay waypoint actual, comprueba distancia y timeout
        if self.current_pose and self.current_waypoint:
            hx = self.current_pose.position.x
            hy = self.current_pose.position.y
            wx, wy = self.current_waypoint
            dist = np.hypot(hx - wx, hy - wy)

            # Si llegó al waypoint, avanza al siguiente
            if dist < 0.5:
                if self.path:
                    self.prev_waypoint = self.current_waypoint
                    self.current_waypoint = self.path.pop(0)
                    self.publish_waypoint(self.current_waypoint)
                    self.waypoint_sent_time = now
                else:
                    self.get_logger().info('Meta alcanzada')
                    self.current_waypoint = None
                    self.waypoint_sent_time = None
                    self.prev_waypoint = None
            else:
                # Si no llegó y pasó el timeout, replanifica desde el waypoint anterior
                if self.waypoint_sent_time and (now - self.waypoint_sent_time > self.replan_timeout):
                    self.get_logger().warn('No se ha llegado al waypoint, volviendo a planificar')
                    if self.goal_pose and self.prev_waypoint:
                        # Simula que el humano está en el waypoint anterior
                        class PrevPose:
                            def __init__(self, x, y):
                                self.position = type('obj', (object,), {'x': x, 'y': y})
                        self.current_pose = PrevPose(self.prev_waypoint[0], self.prev_waypoint[1])
                        self.set_goal(*self.goal_pose)
                        self.current_waypoint = None
                        self.waypoint_sent_time = None
                    elif self.goal_pose:
                        # Si no hay anterior, replanifica desde la posición actual
                        self.set_goal(*self.goal_pose)
                        self.current_waypoint = None
                        self.waypoint_sent_time = None

    # publicador de waypoints

    def publish_waypoint(self, waypoint):
        x, y = waypoint
        goal_msg = AgentGoal()
        goal_msg.agent_id = self.human_id
        goal_msg.goal.position.x = x
        goal_msg.goal.position.y = y
        goal_msg.goal.position.z = 0.0
        goal_msg.goal.orientation.w = 1.0
        goal_msg.radius = 0.3
        self.goal_pub.publish(goal_msg)

    # plubicador de ruta, para Rviz

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    # Función auxiliar creada para disminuir numero de waypoints, finalmente no usada

    def downsample_path(self, path, step=1):
        if not path:
            return []
        new_path = [path[0]]
        for i in range(step, len(path), step):
            new_path.append(path[i])
        if path[-1] != new_path[-1]:
            new_path.append(path[-1])
        return new_path
    

def main(args=None):
    rclpy.init(args=args)
    node = HumanPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
