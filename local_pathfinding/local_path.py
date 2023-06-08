from rclpy.impl.rcutils_logger import RcutilsLogger

from local_pathfinding.ompl_path import OMPLPath


class LocalPath:
    def __init__(self, parent_logger: RcutilsLogger):
        self.logger = parent_logger.get_child(name='local_path')
        self.ompl_path = None
        self.waypoints = None

    def update_if_needed(self):
        ompl_path = OMPLPath(parent_logger=self.logger, max_runtime=1.0)

        if ompl_path.solved:
            self.logger.info('Updating local path')
            self._update(ompl_path)

    def _update(self, ompl_path: OMPLPath):
        self.ompl_path = ompl_path
        self.waypoints = self.ompl_path.get_waypoints()
