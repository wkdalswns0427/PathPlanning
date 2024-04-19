from local_planner.utils.imports import *

class LocalPlanner(object):
    exceed_threshold:float
    finish_threshold:float
    lookahead_dist:float

    optimal_control: np.ndarray
    optimal_poses: np.ndarray

    _path:np.ndarray # (n_waypoints, 3) [[x, y, yaw], ...]
    _obstacles:np.ndarray # (n_obstacles, 3) [[x, y, r], ...]
    _mean_dist_waypoints:float = 0.0

    def __init__(self, lookahead_dist:float=0.6, finish_threshold:float=0.2, exceed_threshold:float=4.0):
        self._is_ready = Event()
        self._path_lock = RLock()
        self._obstacles_lock = RLock()

        self.lookahead_dist = lookahead_dist
        self.finish_threshold = finish_threshold
        self.exceed_threshold = exceed_threshold

        # self.reset() # Child class must call self.reset()

    def reset(self):
        # Updating the controller's parameters - Need to overwite in the child classes
        self._path = np.zeros((0, 3), dtype=np.float64)
        self._obstacles = np.zeros((0, 2), dtype=np.float64)
        self.is_ready = False
        self._mean_dist_waypoints = 0.0


    def get_control(self, pose:np.ndarray, ref_idx:int) -> np.ndarray:
        """
        :param pose: the current pose of the robot [x, y, yaw]
        :param ref_idx: integer index in the reference path
        :return: control (x_dot, y_dot, yaw_dot)
        """
        raise NotImplementedError

    def get_ref_pose(self, pose) -> (int, np.ndarray): # pose
        """
        calculate the closest reference pose and its index in the given global path.
        :param pose: (3, ) the current pose [x, y, yaw]
        :return: (reference index of the reference pose, reference pose)
        """
        with self._path_lock:
            diff = self._path[:, :2] - pose[:2].reshape((1, -1)) # only consider 2D positions
            dist = np.linalg.norm(diff, axis=1)

            ref_idx = min(dist.argmin() + int(self.lookahead_dist / self._mean_dist_waypoints), self._path.shape[0] - 1)
            ref_pose = self._path[ref_idx]

        return ref_idx, ref_pose

    def get_error(self, pose)-> float:
        """
        Calculates the error vector based on a given pose and the global path
        :param pose: the current pose of the robot [x, y, yaw]
        :return: cross track error vector [e_x, e_y]
        """
        yaw = pose[2]
        ref_pose = self.get_ref_pose(pose)[1]
        R = np.array([
            [np.cos(yaw), np.sin(yaw)],
            [-np.sin(yaw), np.cos(yaw)]
        ])
        return R @ (ref_pose[:2] - pose[:2])

    def set_path(self, path:np.ndarray):
        """
        :param path: [[x, y, yaw]], (N_waypoints, 3)
        """
        with self._path_lock:
            self.reset()
            self.is_ready = True

            if path.shape[1] < 3:
                estimated_yaw =  np.zeros((self._path.shape[0],))
                estimated_yaw[:-1] = np.arctan2(path[1:, 0] - path[:-1, 0], path[1:, 1] - path[:-1, 1])
                estimated_yaw[-1] = estimated_yaw[-2]
                self._path = np.hstack((path, estimated_yaw.reshape((-1, 1))))  # (n_rollouts, 3)
            else:
                self._path = path  # (n_waypoints, 3)

            self._mean_dist_waypoints = np.mean(np.linalg.norm(np.diff(self._path[:, :2], axis=0), axis=1))

    def set_obstacles(self, obstacles:np.ndarray):
        assert obstacles.shape[0] == 0 or obstacles.shape[1] == 3
        if obstacles.shape[0] > 0:
            with self._obstacles_lock:
                self._obstacles = obstacles

    def get_path(self):
        return self._path

    def is_path_completed(self, pose, error) -> bool:
        """
        Calculates if the robot reached the end of the global path.
        If the e_x is smaller than the `finish_threshold` or e_y greater than the `exceed threshold`.
        :param pose: the current pose of the robot [x, y, yaw]
        :param error: cross track error vector [e_x, e_y]
        :return: whether the robot reached the end of the global path
        """
        e = np.linalg.norm(error)
        ref_idx = self.get_ref_pose(pose)[0]
        return (ref_idx == self._path.shape[0] - 1) and (e < self.finish_threshold) # or (e > self.exceed_threshold)

    @property
    def is_ready(self) -> bool:
        return self._is_ready.is_set()

    @is_ready.setter
    def is_ready(self, state:bool):
        if state is True:
            self._is_ready.set()
        else:
            self._is_ready.clear()

