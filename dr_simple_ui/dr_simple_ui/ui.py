#!/usr/bin/env python3

import rclpy
import threading

from enum import Enum

from geometry_msgs.msg import TwistStamped
from nicegui import ui, app, ui_run
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

# TODO: Perhaps add limits specific to negative and positive velocities, similar to move_base / Nav2
LINEAR_VELOCITY_STEP_DEFAULT = 0.01
ANGULAR_VELOCITY_STEP_DEFAULT = 0.1
MAX_LINEAR_VELOCITY_DEFAULT = 0.1
MAX_ANGULAR_VELOCITY_DEFAULT = 0.5
CMD_VEL_TOPIC_DEFAULT = "/cmd_vel"


class Command(Enum):
    UP = 1
    LEFT = 2
    RIGHT = 3
    DOWN = 4


class ControlWebUINode(Node):
    def __init__(self):
        super().__init__("control_web_ui_node")

        # Parameters
        self.declare_parameter("cmd_vel_topic", CMD_VEL_TOPIC_DEFAULT)
        self.declare_parameter("max_linear_velocity", MAX_LINEAR_VELOCITY_DEFAULT)
        self.declare_parameter("max_angular_velocity", MAX_ANGULAR_VELOCITY_DEFAULT)
        self.declare_parameter("linear_velocity_step", LINEAR_VELOCITY_STEP_DEFAULT)
        self.declare_parameter("angular_velocity_step", ANGULAR_VELOCITY_STEP_DEFAULT)

        # Holds the current velocity command
        self.velocity_command = TwistStamped()
        self.velocity_command.twist.linear.x = 0.0
        self.velocity_command.twist.angular.z = 0.0

        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped,
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value,
            10,
        )
        self.cmd_vel_timer = self.create_timer(0.5, self.publish_vel_command)

        @ui.page("/")
        def page():
            with ui.card():
                with ui.column().classes("items-center gap-4"):
                    ui.button(
                        "⇧",
                        color="green",
                        on_click=lambda: self.update_vel_command(Command.UP),
                    )

                    with ui.row().classes("gap-4"):
                        ui.button(
                            "⇦",
                            color="green",
                            on_click=lambda: self.update_vel_command(Command.LEFT),
                        )
                        ui.button(
                            "⇩",
                            color="green",
                            on_click=lambda: self.update_vel_command(Command.DOWN),
                        )
                        ui.button(
                            "⇨",
                            color="green",
                            on_click=lambda: self.update_vel_command(Command.RIGHT),
                        )
                    ui.button(
                        "STOP",
                        color="red",
                        on_click=lambda: self.set_vel_command(linear=0.0, angular=0.0),
                    )
                    ui.label("Velocity Control")

    def update_vel_command(self, command: Command) -> None:
        if command == Command.UP:
            self.velocity_command.twist.linear.x = min(
                self.velocity_command.twist.linear.x
                + self.get_parameter("linear_velocity_step")
                .get_parameter_value()
                .double_value,
                self.get_parameter("max_linear_velocity")
                .get_parameter_value()
                .double_value,
            )
        elif command == Command.LEFT:
            self.velocity_command.twist.angular.z = min(
                self.velocity_command.twist.angular.z
                + self.get_parameter("angular_velocity_step")
                .get_parameter_value()
                .double_value,
                self.get_parameter("max_angular_velocity")
                .get_parameter_value()
                .double_value,
            )
        elif command == Command.RIGHT:
            self.velocity_command.twist.angular.z = max(
                self.velocity_command.twist.angular.z
                - self.get_parameter("angular_velocity_step")
                .get_parameter_value()
                .double_value,
                -self.get_parameter("max_linear_velocity")
                .get_parameter_value()
                .double_value,
            )
        elif command == Command.DOWN:
            self.velocity_command.twist.linear.x = max(
                self.velocity_command.twist.linear.x
                - self.get_parameter("linear_velocity_step")
                .get_parameter_value()
                .double_value,
                -self.get_parameter("max_linear_velocity")
                .get_parameter_value()
                .double_value,
            )
        self.get_logger().info(
            f"Current velocity command: [ linear: ( {self.velocity_command.twist.linear.x} m/s ), angular: ( {self.velocity_command.twist.angular.z} rad/s ) ]"
        )

    def set_vel_command(self, linear: float, angular: float) -> None:
        self.get_logger().info(
            f"Current velocity command: [ linear: ( {linear} m/s ), angular: ( {angular} rad/s ) ]"
        )
        self.velocity_command.twist.linear.x = linear
        self.velocity_command.twist.angular.z = angular

    def publish_vel_command(self) -> None:
        self.cmd_vel_publisher.publish(self.velocity_command)


"""
The following structure to start up a NiceGUI along ROS2 is based on the example at:
https://github.com/zauberzeug/nicegui/tree/main/examples/ros2
"""


def main() -> None:
    # NOTE: This is ROS' entry point as set in setup.py, it es empty to permit NiceGUI's auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = ControlWebUINode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        node.destroy_node()
        # rclpy is already shutdown here, so there is no need to shut it down explictely (?)
        # rclpy.shutdown()


# Initialize ROS on a separate thread after NiceGUI starts
app.on_startup(lambda: threading.Thread(target=ros_main).start())

# Tells uvicorn where the FastAPI is (<ros_node_name>:app)
# -> Basically avoids re-importing the module and causing Context.init() crashes
# (usually due to multiple rclpy.init() calls)
ui_run.APP_IMPORT_STRING = f"{__name__}:app"
ui.run("0.0.0.0", port=8080)  # Available from other devices on the network
