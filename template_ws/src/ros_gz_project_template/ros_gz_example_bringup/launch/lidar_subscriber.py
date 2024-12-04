#!/usr/bin/env python3
import rclpy
import subprocess
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient


class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver')
        # Suscripción al tópico del LIDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/diff_drive/scan',  # Tópico del LIDAR
            self.lidar_callback,
            10
        )

        # Subscripción al tópico de odometría
        self.subscription = self.create_subscription(
            Odometry,
            '/diff_drive/odometry', 
            self.odometry_callback,
            10
        )

        # Publicador para comandos de velocidad
        self.publisher = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)

        # Parametros de navegacion
        self.start_x = 0.0
        self.start_y = 0.0
        # Objetivos
        self.goal_x = -15.0
        self.goal_y = 19.0
 
        # Parámetros de control
        self.desired_distance_to_wall = 2.5   # Distancia deseada a la pared (metros)
        self.linear_speed = 0.9             # Velocidad lineal (m/s)
        self.angular_speed = 0.5            # Velocidad angular (rad/s)

        # Estado del robot
        self.following_wall = False         # Indica si ya detectó una pared
        self.last_right_distance = None     # Última distancia registrada a la pared derecha
        self.en_movimiento = True           # Indica si el robot está en movimiento

        # Mapa generado
        self.mapa_generado = False

    def odometry_callback(self, msg):
        # Obtener la posición actual del robot
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Calcular distancia al objetivo
        distance_to_goal = abs(self.goal_x - x) + abs(self.goal_y - y) 

        # self.get_logger().info('Entro al odometry_callback, distancia: {}'.format(distance_to_goal))    

        # Si el robot llegó al objetivo, detenerlo
        if distance_to_goal < 2:
            self.en_movimiento = False
            if not self.mapa_generado:
                self.guardar_mapa()
                self.mapa_generado = True
        else:
            self.en_movimiento = True

    def guardar_mapa(self):
        try:
            subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', 'Mapa'], check=True)
            self.get_logger().info('Mapa guardado en /root/template_ws/Mapa.pgm.')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Error al guardar el mapa: {e}')
      
    
    
    def lidar_callback(self, msg):
        # Rango limitado del LIDAR: -80° a 80° aproximadamente
        ranges = msg.ranges
        valid_ranges = [d if msg.range_min < d < msg.range_max else float('inf') for d in ranges]

        # Dividir rangos en sectores: derecha, frente, izquierda
        num_readings = len(ranges)
        right_sector = valid_ranges[:num_readings // 3]   # Derecha
        front_sector = valid_ranges[num_readings // 3:2 * num_readings // 3]  # Frente
        left_sector = valid_ranges[2 * num_readings // 3:]  # Izquierda

        # Calcular distancias mínimas en cada sector
        right_distance = min(right_sector) if right_sector else float('inf')
        front_distance = min(front_sector) if front_sector else float('inf')
        left_distance = min(left_sector) if left_sector else float('inf')

        cmd = Twist()

        # Imprimir el estado de self.following_wall
        # self.get_logger().info('Siguiendo pared: {}'.format(self.following_wall))

        if self.en_movimiento:
            # Lógica de navegación
            if not self.following_wall and front_distance > self.desired_distance_to_wall and right_distance > self.desired_distance_to_wall:
                # Avanzar hasta encontrar una pared
                self.spinning = 0
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0
                # self.get_logger().info('Avanzando hacia adelante, buscando pared.')
            elif not self.following_wall and right_distance <= self.desired_distance_to_wall:
                self.spinning = 0
                # Detecta la primera pared a la derecha
                self.following_wall = True
                self.last_right_distance = right_distance
                # self.get_logger().info('Primera pared detectada. Siguiendo la pared derecha.')
            elif self.following_wall and right_distance > self.desired_distance_to_wall:
                self.spinning = self.spinning + 1
                # Perdió la pared derecha, girar a la derecha para recuperarla
                cmd.linear.x = 0.4
                cmd.angular.z = -self.angular_speed
                # self.get_logger().info('Pared derecha perdida, girando para recuperarla.')
            elif (self.following_wall and (front_distance < self.desired_distance_to_wall) and (right_distance < self.desired_distance_to_wall)):
                # Pared enfrente, girar a la izquierda
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed
                # self.get_logger().info('Pared enfrente, girando a la izquierda.')
            else:
                # Avanzar paralelo a la pared derecha
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0
                # self.get_logger().info('Siguiendo la pared derecha.')


            # Si el robot está siguiendo la pared, actualiza la última distancia conocida
            if right_distance <= self.desired_distance_to_wall * 2:
                self.last_right_distance = right_distance

            # Publicar comando de movimiento
            self.publisher.publish(cmd)
        else:
            # Detener el robot
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = 0.0
            self.publisher.publish(cmd)


def main():
    rclpy.init()
    node = MazeSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
