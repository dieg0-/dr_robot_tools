from battery_handler_base import BatteryHandlerBase

from rclpy.impl.rcutils_logger import RcutilsLogger

class DummyHandler(BatteryHandlerBase):
    def handle(self, logger: RcutilsLogger, voltage: float):
        logger.info(f"Current battery state: {voltage:.3} V")