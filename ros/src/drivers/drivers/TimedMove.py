# timed_mover.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TimedMoverNode(Node):
    """
    Um nó ROS2 que publica um comando de velocidade por um tempo determinado e depois para.

    Este nó publica uma mensagem Twist no tópico /cmd_vel com valores de velocidade
    linear.x e angular.z especificados. Ele faz isso a cada 'dt' segundos.
    Após uma 'duration' total em segundos, ele publica um comando de parada (velocidades nulas)
    uma única vez e desativa seu timer.
    """
    def __init__(self):
        """
        Construtor da classe TimedMoverNode.
        Inicializa os parâmetros, o publisher, o timer e as variáveis de controle.
        """
        super().__init__('timed_mover_node')

        # --- Declaração de Parâmetros com valores padrão ---
        # Isso permite que os valores sejam alterados no momento da execução.
        # Ex: ros2 run <pkg> <node> --ros-args -p linear_x:=0.5 -p duration:=5.0
        self.declare_parameter('linear_x', 0.0)      # m/s
        self.declare_parameter('angular_z', 1.0)     # rad/s
        self.declare_parameter('dt', 0.01)            # s (frequência de publicação de 10Hz)
        self.declare_parameter('duration', 0.45)      # s (tempo total de movimento)
        self.declare_parameter('sectors', 6.0) #90 graus

        # --- Obtenção dos valores dos parâmetros ---
        self._linear_x = self.get_parameter('linear_x').get_parameter_value().double_value
        self._angular_z = self.get_parameter('angular_z').get_parameter_value().double_value
        self._dt = self.get_parameter('dt').get_parameter_value().double_value
        self._duration = self.get_parameter('duration').get_parameter_value().double_value
        
        # --- Cálculo de quantos 'ticks' do timer são necessários ---
        # Usamos max(1, ...) para garantir que ele rode pelo menos uma vez se a duração for pequena
        try:
            self._stop_count = max(1, int(self._duration / self._dt))
        except ZeroDivisionError:
            self.get_logger().error("'dt' não pode ser zero! Definindo _stop_count como 1.")
            self._stop_count = 1
            
        self._current_count = 0
        self.sectors_count = self.get_parameter('sectors').get_parameter_value().double_value

        # --- Criação do Publisher ---
        # Publica mensagens do tipo Twist no tópico /cmd_vel
        # O '10' é o tamanho da fila (QoS - Quality of Service)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Criação do Timer ---
        # O timer chama a função 'timer_callback' a cada '_dt' segundos
        self.timer_ = self.create_timer(self._dt, self.timer_callback)

        self.get_logger().info(
            f"Nó 'timed_mover' iniciado. Movendo com linear.x={self._linear_x} "
            f"e angular.z={self._angular_z} por {self._duration} segundos."
        )
        self.get_logger().info(f"O comando de parada será enviado após {self._stop_count} publicações.")


    def timer_callback(self):
        """
        Função chamada periodicamente pelo timer.
        Publica a mensagem de movimento ou a de parada, dependendo da contagem.
        """
        # Cria uma nova mensagem Twist a cada chamada
        twist_msg = Twist()

        if self._current_count < self._stop_count:
            # --- Fase de Movimento ---
            twist_msg.linear.x = self._linear_x
            twist_msg.angular.z = self._angular_z
            self.publisher_.publish(twist_msg)
            self.get_logger().info(f'Publicando movimento... Contagem: {self._current_count + 1}/{self._stop_count}')

        elif self._current_count >= self._stop_count and self.sectors_count > 1:
            self._current_count = -1
            self.sectors_count -= 1
            # time.sleep(0.6)
        
        else:
            # --- Fase de Parada ---
            # O Twist() já é inicializado com todos os campos zerados (linear.x=0.0, angular.z=0.0)
            self.publisher_.publish(twist_msg)
            self.get_logger().info('Duração atingida. Publicando comando de parada (velocidades nulas).')
            
            # --- Desativa o timer ---
            # Isso é crucial para que ele pare de publicar e usar recursos.
            self.timer_.cancel()
            self.get_logger().info('Timer desativado. O nó permanecerá ativo mas inerte.')

        # Incrementa o contador para o próximo ciclo
        self._current_count += 1


def main(args=None):
    """Função principal que inicializa e executa o nó ROS."""
    rclpy.init(args=args)
    
    timed_mover_node = TimedMoverNode()
    
    # rclpy.spin() mantém o nó vivo para que os callbacks (como o do timer) possam ser processados.
    rclpy.spin(timed_mover_node)
    
    # Destroi o nó explicitamente
    # (opcional, pois a destruição é automática na saída do 'with')
    timed_mover_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()