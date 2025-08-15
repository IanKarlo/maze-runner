# timed_mover.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from project_interfaces.srv import TimedMoveResponse
import time
from math import fabs

'''
float64 distance
float64 angle
float64 orientation_sign
---
'''

time_for_angle = {
    '0': 0,
    '15': 0.4,
    '30': 0.84,
    '45': 1.25,
    '60': 1.7,
    '75': 2.15,
    '90': 2.58,
}
valid_angles = [0, 15, 30, 45, 60, 75, 90]

time_for_distance = {
    '0': 0,
    '0.5': 3.09,
    '0.25': 1.54,
    '0.125': 0.77
}
valid_distances = [0, 0.5, 0.25, 0.125]

rotation_twist = Twist()
rotation_twist.linear.x = 0.0
rotation_twist.angular.z = 1.0

translation_twist = Twist()
translation_twist.linear.x = 0.2
translation_twist.angular.z = 0.03


def convert_movement_to_times(angle, distance):
    """
    Converte ângulo e distância para tempos de rotação e translação.
    Seleciona o valor mais próximo das tabelas definidas.
    """
    # ângulo mais próximo
    nearest_angle = min(valid_angles, key=lambda x: fabs(x - angle))
    rotation_time = time_for_angle[str(nearest_angle)]

    # distância mais próxima
    nearest_distance = min(valid_distances, key=lambda x: fabs(x - distance))
    print(f'----- {nearest_distance} - {str(nearest_distance)} - {time_for_distance.keys()}')
    translation_time = time_for_distance[str(nearest_distance)]

    return rotation_time, translation_time


class TimedMoverNode(Node):
    def __init__(self):
        super().__init__('timed_move_service')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.srv = self.create_service(
            TimedMoveResponse,
            'send_move_command',
            self.send_move_command_callback
        )

        self.get_logger().info("Serviço 'send_move_command' pronto para receber comandos.")

    def timed_move(self, twist, duration):
        """
        Publica o twist continuamente a cada 0.01s por 'duration' segundos,
        depois publica um twist zerado para parar o robô.
        """
        end_time = time.time() + duration
        while time.time() < end_time:
            # print('PUBLICANDO', time.time(), end_time )
            self.publisher_.publish(twist)
            time.sleep(0.01)

        # Para o robô
        stop_twist = Twist()
        self.publisher_.publish(stop_twist)

    def send_move_command_callback(self, request, response):
        angle = request.angle
        distance = request.distance
        orientation = request.orientation_sign

        self.get_logger().info(
            f"Recebido comando: angle={angle}, distance={distance}, orientation_sign={orientation}"
        )

        rotation_time, translation_time = convert_movement_to_times(angle, distance)

        # Movimento de rotação
        rot_twist = Twist()
        rot_twist.linear.x = rotation_twist.linear.x
        rot_twist.angular.z = rotation_twist.angular.z * orientation * -1

        self.timed_move(rot_twist, rotation_time)

        # Espera 250 ms
        time.sleep(0.25)

        # Movimento de translação
        self.timed_move(translation_twist, translation_time)

        self.get_logger().info("Movimento concluído.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TimedMoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
