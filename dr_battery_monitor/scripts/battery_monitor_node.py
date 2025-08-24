#!/usr/bin/env python3

import asyncio
import rclpy
import time

from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from config_parser import BatteryRange, ConfigParser
from plugin_handler import load_plugins
# from desktop_notifier import DesktopNotifier, Notification

# Battery thresholds (Volts)
NORMAL_VOLTAGE: float = 11.5
MID_VOLTAGE: float = 11.1
WARNING_VOLTAGE: float = 10.5
CRITICAL_VOLTAGE: float = 10.1



class BatteryMonitor(Node):
    def __init__(self):
        super().__init__("battery_monitor")

        # Parameters
        self.declare_parameter("normal_voltage", NORMAL_VOLTAGE)
        self.declare_parameter("mid_voltage", MID_VOLTAGE)
        self.declare_parameter("warning_voltage", WARNING_VOLTAGE)
        self.declare_parameter("critical_voltage", CRITICAL_VOLTAGE)

        self.config_parser = ConfigParser()
        self.config = self.config_parser.load("default_config.yaml")
        print(self.config)
        self.handlers = load_plugins()
        self.current_range = BatteryRange.NORMAL
        self.last_notification = time.time()
        # TODO
        self.time_lapse = 1
        # self.notifier = DesktopNotifier()
        self.battery_sub = self.create_subscription(
            BatteryState, "battery_state", self.battery_sub_clbk, qos_profile=10
        )
    
    # async def send_notification(self, notifier: DesktopNotifier, msg: str):
    #     await notifier.send(
    #         title="Battery Report",
    #         message=msg
    #     )

    def handle_normal_voltage(self):
        pass

    def handle_mid_voltage(self):
        # TODO
        self.get_logger().info(
            "The battery state has reached a medium level. Consider wrapping-up soon."
        )
        print(self.dummy_flag)
        msg: str = "The battery state has reached a medium level. Consider wrapping-up soon."
        # if not self.dummy_flag:
        #     print("Will notify!")
        #     self.dummy_flag = True
        #     asyncio.run(self.send_notification(self.notifier, msg))
        

    def handle_warning_voltage(self):
        # TODO
        self.get_logger().warn("The battery state is low, please recharge the robot.")

    def handle_critical_voltage(self):
        # TODO
        self.get_logger().warn(
            "The battery state is at a critical level, wrap-up and turn-off the robot now to avoid permanent damage to the cells!"
        )

    def battery_sub_clbk(self, battery_state: BatteryState):
        print(f"Battery percentage: {battery_state.percentage:.3} %")
        normal_voltage: float = (
            self.get_parameter("normal_voltage").get_parameter_value().double_value
        )
        mid_voltage: float = (
            self.get_parameter("mid_voltage").get_parameter_value().double_value
        )
        warning_voltage: float = (
            self.get_parameter("warning_voltage").get_parameter_value().double_value
        )
        critical_voltage: float = (
            self.get_parameter("critical_voltage").get_parameter_value().double_value
        )
        if (state := battery_state.voltage) <= critical_voltage:
            self.handle_critical_voltage()
            return

        if state <= warning_voltage:
            self.handle_warning_voltage()
            return

        if state <= mid_voltage:
            self.handle_mid_voltage()
            return

        self.get_logger().debug(
            f"The current voltage {state} V > normal minimum {normal_voltage} V and thus within the ideal working range."
        )
        # self.handle_normal_voltage()
        self.handlers["DummyHandler"].handle(self.get_logger(), state)


def main(args=None):
    rclpy.init()
    monitor_node = BatteryMonitor()
    try:
        rclpy.spin(monitor_node)
    except KeyboardInterrupt:
        print("Terminating node on request...")
        monitor_node.destroy_node()


if __name__ == "__main__":
    main()
