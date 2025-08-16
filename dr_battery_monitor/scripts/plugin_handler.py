import importlib
import inspect
import os

from ament_index_python.packages import get_package_share_directory
from battery_handler_base import BatteryHandlerBase


def load_plugins():
    package_dir = get_package_share_directory("dr_battery_monitor")
    plugins_dir = os.path.join(package_dir, "handler_plugins")

    handlers = {}
    for f in [
        file_name for file_name in os.listdir(plugins_dir) if file_name != "__init__.py"
    ]:
        name, _ = os.path.splitext(f)
        name = f"handler_plugins.{name}"
        module = importlib.import_module(name)
        for member_name, member in inspect.getmembers(module, inspect.isclass):
            if (
                issubclass(member, BatteryHandlerBase)
                and member is not BatteryHandlerBase
            ):
                print(f"- Found plugin: {member_name}")
                handlers[member_name] = member()
    return handlers
