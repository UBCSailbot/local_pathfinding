from rclpy.impl.rcutils_logger import RcutilsLogger

from local_pathfinding.ompl_path import OMPLPath


class LocalPath:
    def __init__(self, parent_logger: RcutilsLogger):
        self.logger = parent_logger.get_child(name='local_path')
        self.ompl_path = OMPLPath(parent_logger=self.logger)

    def update_if_needed(self):
        self.ompl_path.plan()
