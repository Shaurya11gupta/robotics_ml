#!/usr/bin/env python3



import rclpy
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped 
import numpy as np


class SimpleController(Node):

    def __init__(self):
        super().__init__('simple_controller')
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            "mecanum_bot_controller/cmd_vel",
            self.velocity_callback,
            10)  


        self.wheel_pub_1 = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)

    def velocity_callback(self, msg):
    
        width = {"fr": 0.275, "fl": 0.275, "rr": 0.275, "rl": 0.275}
        length = {"fr": 0.575, "fl": 0.575, "rr": 0.575, "rl": 0.575}

        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        wz = msg.twist.angular.z

        mat = np.array([[1, 1, (width["fr"] + length["fr"])],
                     [1, -1, -(width["fl"] + length["fl"])],
                     [1, -1, (width["rr"] + length["rr"])],
                     [1, 1, -(width["rl"] + length["rl"])]]) 
         
        cmd_vel = np.array([vx,vy,wz])
        wheel_vel = np.matmul(mat, cmd_vel.T)



        wheel_vel_msg = Float64MultiArray()

        wheel_vel_msg.data = [wheel_vel[0],wheel_vel[1],wheel_vel[2],wheel_vel[3]]
        self.wheel_pub_1.publish(wheel_vel_msg)


        

def main():
    rclpy.init()
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
