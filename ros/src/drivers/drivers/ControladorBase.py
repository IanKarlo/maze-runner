#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from drivers.motor import pd
from drivers.motor import motors_driver
from drivers.motor import gpio as gp
import math

left_feedback_accumulator = [0,0,0,0,0]
right_feedback_accumulator = [0,0,0,0,0]
L = 0.225 #wheel distance
d = L/2 #leia em cima
r = 0.065/2 #wheel radius
pulses_per_revolution = 1000
direction_r = 1 # 1 forward | -1 backward
direction_l = 1

def map_range(x, in_min=-40, in_max=40, out_min=-100, out_max=100):
    return ( (x - in_min) * (out_max - out_min) / (in_max - in_min) ) + out_min

def calculate_velocity_in_pulses(v, w):
    ul = (v - d*w)/r
    ur = (v + d*w)/r
    ul_pulses = ul*(pulses_per_revolution/(2*math.pi))
    ur_pulses = ur*(pulses_per_revolution/(2*math.pi))

    return ul_pulses, ur_pulses

def control_loop(desired_velocity, measured_velocity, pd, accumulator_array, direction):
    accumulator_array.append(measured_velocity)
    if len(accumulator_array) > 5:
        accumulator_array.pop(0)
    measured_mean = sum(accumulator_array)/(len(accumulator_array))

    control = pd.compute(desired_velocity, measured_mean)
    control = map_range(control)
    if direction == 1:
        return 100 if control > 100 else 0 if control < 0 else control
    else:
        return -100 if control < -100 else 0 if control > 0 else control


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor')
        self.motor_driver = motors_driver.MotorDriver(lambda chip, gpio, level, timestamp: self.get_pulses_encoders(chip, gpio, level, timestamp))

        self.left_pd = pd.PDController(3, 2.5)
        self.right_pd = pd.PDController(2.8, 2.5)
        self.ds = 0.02 # 10 ms

        self.stop_counter = 0
        self.stop_time = 5

        self.v_setpoint = 0.0 # Velocidade linear desejada (m/s)
        self.w_setpoint = 0.0 # Velocidade angular desejada (rad/s)
        self.ul_setpoint = 0.0
        self.ur_setpoint = 0.0

        self.ul_measured = 0
        self.ur_measured = 0

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10 # QoS profile
        )
        
    
        self.timer = self.create_timer(self.ds, self.control_loop_callback)
        


    def get_pulses_encoders(self, chip, gpio, level, timestamp):
    # Motor direito
        if gpio == 20 or gpio == 21:
            self.ur_measured += 1 * direction_r

        # Motor esquerdo
        if gpio == 6 or gpio == 5:
            self.ul_measured += 1 * direction_l


    def cmd_vel_callback(self, msg):
        # Twist.linear.x -> v
        # Twist.angular.z -> w
        global direction_l
        global direction_r

        self.v_setpoint = msg.linear.x
        self.w_setpoint = msg.angular.z
        ul, ur = calculate_velocity_in_pulses(self.v_setpoint, self.w_setpoint)
        self.ul_setpoint = ul * self.ds
        self.ur_setpoint = ur * self.ds
        direction_l = 1 if ul >= 0 else -1
        direction_r = 1 if ur >= 0 else -1

        self.stop_counter = 0           


    def control_loop_callback(self):
        ul_measured = self.ul_measured
        ur_measured = self.ur_measured
        ul_setpoint = self.ul_setpoint
        ur_setpoint = self.ur_setpoint

        self.ul_measured = 0
        self.ur_measured = 0

        pwm_l = control_loop(ul_setpoint, ul_measured, self.left_pd, left_feedback_accumulator, direction_l)
        pwm_r = control_loop(ur_setpoint, ur_measured, self.right_pd, right_feedback_accumulator, direction_r)

        self.motor_driver.run_motors(pwm_r, pwm_l)

        print(f'ul_measured: {ul_measured} - ur_measured: {ur_measured} ----- ul_setpoint: {ul_setpoint} - ur_setpoint: {ur_setpoint} - pwm_l = {pwm_l} - pwm_r = {pwm_r}')
         

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()