#!/usr/bin/env python3

import rclpy
import threading

from nicegui import ui, app, ui_run
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


class ControlWebUINode(Node):
    def __init__(self):
        super().__init__("control_web_ui_node")
        self.publisher = self.create_publisher(String, "message", 10)

        @ui.page("/")
        def page():
            with ui.card():
                with ui.column().classes("items-center gap-4"):
                    ui.button(
                        "⇧", color="green", on_click=lambda: self.publish_command("Up")
                    )

                    with ui.row().classes("gap-4"):
                        ui.button(
                            "⇦",
                            color="green",
                            on_click=lambda: self.publish_command("Left"),
                        )
                        ui.button(
                            "⇩",
                            color="green",
                            on_click=lambda: self.publish_command("Down"),
                        )
                        ui.button(
                            "⇨",
                            color="green",
                            on_click=lambda: self.publish_command("Right"),
                        )

                    ui.label("Velocity Control")

    def publish_command(self, m: str) -> None:
        self.get_logger().info("Publishing!")
        msg = String()
        msg.data = m
        self.publisher.publish(msg)


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
ui_run.APP_IMPORT_STRING = f"{__name__}:app"  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run("0.0.0.0", port=8080)  # Available from other devices on the network
