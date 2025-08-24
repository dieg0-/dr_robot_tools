import os
import yaml

from enum import Enum

from ament_index_python.packages import get_package_share_directory


class BatteryRange(Enum):
    NORMAL = 1
    MID = 2
    WARNING = 3
    CRITICAL = 4


class ConfigParser:
    def __init__(self):
        package_dir = get_package_share_directory("dr_battery_monitor")
        self.configs_dir = os.path.join(package_dir, "config")

    def load(self, config_file):
        config = {}
        with open(f"{self.configs_dir}/{config_file}", "r") as data:
            config = yaml.safe_load(data)
        return config
