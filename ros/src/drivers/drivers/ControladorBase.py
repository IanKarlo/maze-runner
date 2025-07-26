#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from motor import pd
from motor import motors_driver

class MotorControllerNode(Node):
    def __init__(self):
        self.motor_driver = motors_driver.MotorDriver(lambda chip, gpio, level, timestamp: self.get_pulses_encoders(chip, gpio, level, timestamp))

        self.linear_pd = pd.PDController(1, 1)
        self.angular_pd = pd.PDController(1, 1)
        self.ds = 0.01 # 10 ms

        self.v_setpoint = 0.0 # Velocidade linear desejada (m/s)
        self.w_setpoint = 0.0 # Velocidade angular desejada (rad/s)

        self.u_l = 0
        self.u_r = 0

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10 # QoS profile
        )
        
    
        self.timer = self.create_timer(self.ds, self.control_loop_callback)
        
        pass

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

    def control_loop_callback(self):
        # zera no início ou no f
        u_l = self.u_l
        u_r = self.u_r
        self.u_l = 0
        self.u_r = 0

        v = (u_r + u_l)/2
        w = (u_r - u_l)/2
        
        control_v = self.linear_pd.compute(self.v_setpoint, v)
        control_w = self.angular_pd.compute(self.w_setpoint, w)

        signal_r = control_v + control_w
        signal_l = control_v - control_w

        pwm_r = 100 if signal_r > 100 else -100 if signal_r < -100 else signal_r
        pwm_l = 100 if signal_l > 100 else -100 if signal_l < -100 else signal_l

        self.motor_driver.run_motors(pwm_r, pwm_l)
        pass
         

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
