from abc import ABC, abstractmethod

from rclpy.impl.rcutils_logger import RcutilsLogger

class BatteryHandlerBase(ABC):
    @abstractmethod
    def handle(self, logger: RcutilsLogger, voltage: float):
        pass