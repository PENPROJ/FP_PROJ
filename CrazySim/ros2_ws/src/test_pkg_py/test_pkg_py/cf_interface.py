#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String
from crazyflie_py import Crazyswarm

class CFWrapper:
    def __init__(self):
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.allcfs = self.swarm.allcfs
        self.cf = self.allcfs.crazyflies[0]

        self.node = rclpy.create_node('cf_interface')
        self.subscription = self.node.create_subscription(
            String,
            'keyboard_input',
            self.keyboard_callback,
            10
        )
        self.node.get_logger().info("cf_interface node ready.")

    def keyboard_callback(self, msg):
        if not msg.data:
            return
        input_char = msg.data[0]
        if input_char == 'p':
            self.cf.arm(True)
            self.node.get_logger().info("ARM command sent.")
        elif input_char == 'o':
            self.cf.arm(False)
            self.node.get_logger().info("DISARM command sent.")

    def spin(self):
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    wrapper = CFWrapper()
    wrapper.spin()

if __name__ == '__main__':
    main()

