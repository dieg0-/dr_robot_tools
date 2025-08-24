#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class BatterySim(Node):
    def __init__(self):
        super().__init__("battery_sim")

        self.initial_voltage: float = 12.5
        self.current_voltage: float = self.initial_voltage
        self.pub = self.create_publisher(BatteryState, "battery_state", qos_profile=10)
        self.timer = self.create_timer(3.0, self.publish_battery_state)

    def publish_battery_state(self):
        self.current_voltage = max(self.current_voltage - 0.005, 0.0)
        battery_state: BatteryState = BatteryState()
        battery_state.voltage = self.current_voltage
        self.pub.publish(battery_state)

def main(args=None):
    rclpy.init()
    battery_sim_node: BatterySim = BatterySim()
    print("Simulating Battery State!")
    try:
        rclpy.spin(battery_sim_node)
    except KeyboardInterrupt:
        print("The node has been stopped on request!")

if __name__ == "__main__":
    main()