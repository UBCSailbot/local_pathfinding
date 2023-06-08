from rclpy.impl.rcutils_logger import RcutilsLogger

from local_pathfinding.ompl_path import OMPLPath


class LocalPath:
    def __init__(self, logger: RcutilsLogger):
        self.logger = logger
        self.ompl_path = OMPLPath(self.logger)

    def update_if_needed(self):
        self.ompl_path.plan()
