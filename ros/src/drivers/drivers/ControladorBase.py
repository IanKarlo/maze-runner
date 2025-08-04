#!/usr/bin/env python3
import time
from venv import logger
from drivers.motor.gpio import digital_read
import rclpy
from math import fabs
import logging
from rclpy.node import Node
from geometry_msgs.msg import Twist
from drivers.motor import pd
from drivers.motor import motors_driver

# time_cummulative = []
class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor')
        self.motor_driver = motors_driver.MotorDriver(lambda chip, gpio, level, timestamp: self.get_pulses_encoders(chip, gpio, level, timestamp))

        # Calcular quantos pulsos os encoders contam em 1 ds com PWM 100, (ou seja, essa será a v_max (pulse/ds) ).
        # o teste: rodar por alguns segundos imprimindo DS, e ver os resultados do meio. Os do início e final ignora.
        # sabendo disso, só faz sentido botar valores de v_setpoint abaixo desse valor máximo e o Ks dos PDs tem q serem geralmente limitados a esse v_max.
        # p/ ds = 0.02s deu em volta de 32~36 pulsos por ds 
        # Se tiver muito abaiixo ou muito alto, o K não está bom
        # Mandar o robô ir pra frente, e ir ajustando apenas o KP, quando chegar num valor estável onde v = v_setpoint. 
        # Aí começa a mexer no KD pra diminuir a variação de v com o que o KP conseguiu
        # Depois faz o robô girar em torno do próprio eixo e vai mudando o KP angular. Quando chegar num valor semelahnte
        # Resumindo. Vai saber que o PD tá bom quando vc mandar andar X e a velocidade medida dele se mantrr bem próxima de X
        self.linear_pd = pd.PDController(15, 0)
        self.angular_pd = pd.PDController(0, 0)
        self.ds = 0.02 # 10 ms, tempo de processamento de um ciclo da malha de controle

        self.stop_counter = 0
        self.stop_time = 0.1 

        self.v_setpoint = 0.0 # Velocidade linear desejada (m/s)
        self.w_setpoint = 0.0 # Velocidade angular desejada (rad/s)

        self.u_l = 0
        self.u_r = 0

        self.motorA_state_enc1 = 0
        self.motorA_state_enc2 = 0

        self.motorB_state_enc1 = 0
        self.motorB_state_enc2 = 0

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10 # QoS profile
        )
        
    
        self.timer = self.create_timer(self.ds, self.control_loop_callback)
        

    def get_pulses_encoders(self, chip, gpio, level, timestamp):
        if gpio == 20 or gpio == 21:
            self.u_r += 1
            return
        
        if gpio == 5 or gpio == 6:
            self.u_l += 1
            return


    def cmd_vel_callback(self, msg):
        # Twist.linear.x -> v
        # Twist.angular.z -> w
        self.v_setpoint = msg.linear.x
        self.w_setpoint = msg.angular.z

        self.stop_counter = 0           

    def control_loop_callback(self):
        # zera no início ou no f
        u_l = self.u_l
        u_r = self.u_r
        self.u_l = 0
        self.u_r = 0

        v = (u_r + u_l)/2.0
        w = (u_r - u_l)/2.0
        
        control_v = self.linear_pd.compute(self.v_setpoint, v) # sinal de controle de ajuste da vel. linear
        control_w = self.angular_pd.compute(self.w_setpoint, w) # sinal de controle de ajuste da vel. angular

        signal_r = control_v + control_w # sinal de "PWM" a ser usado no motor direito
        signal_l = control_v - control_w # sinal de "PWM" a ser usado no motor esquerdo

        pwm_r = 100 if signal_r > 100 else -100 if signal_r < -100 else signal_r
        pwm_l = 100 if signal_l > 100 else -100 if signal_l < -100 else signal_l

        # each HIGH means a transition, thus a pulse
        self.motorA_state_enc1 += digital_read(20)
        self.motorA_state_enc2 += digital_read(21)

        self.motorB_state_enc1 += digital_read(5)
        self.motorB_state_enc2 += digital_read(6)


        # right and left DC wheel speed , translational and rotation velocity
        logger.info(
    "\nu_r: %s\nu_l: %s\nv: %s\nw: %s\nsignal_r: %s\nsignal_l: %s\n",
    u_r, u_l, v, w, signal_r, signal_l
        )

        self.motor_driver.run_motors(pwm_r, pwm_l)
        self.stop_counter += self.ds
        if self.stop_counter >= self.stop_time:
            start = time.time()
            end = time.time()
            self.stop_counter = 0
            self.v_setpoint = 0
            self.w_setpoint = 0
            logger.info(
                    "\npulses_enc1_motorA: %s\npulses_enc2_motorA: %s\npulses_enc1_motorB: %s\npulses_enc2_motorB: %s,took: %s s\n",
        self.motorA_state_enc1, self.motorA_state_enc2, self.motorB_state_enc1, self.motorB_state_enc2, end-start) #ans in ms

        # USADO APENAS EM TESTES PARA FAZER O MOTOR PARAR APÓS stop_time SEGUNDOS. NÃO FAZ PARTE DO CÓDIGO.

def main(args=None):
    try:
        rclpy.init(args=args)
        node = MotorControllerNode()
        node.motor_driver.run_motors(0,0) 
        logging.basicConfig(filename="ds.log",level=logging.INFO,  
                            format='%(asctime)s - %(message)s',
                            datefmt='%m-%d %H:%M:%S')
        logger.info('started\n')
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt as h:
        node.motor_driver.run_motors(0,0) 
        ans = input("Quer dar uma descricao para o log?[y/n]")
        if ans == "y" or ans == "Y":
            with open('ds.log', 'a') as file: # append mode
                desc = input("Nome do log: ")
                file.write(f"desc: {desc}")
        logger.info('finished\n')
    

if __name__ == '__main__':
        main()


