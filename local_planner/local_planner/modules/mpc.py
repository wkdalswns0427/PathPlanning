from local_planner.utils.imports import *
from local_planner.modules.local_planner import LocalPlanner
import local_planner.modules.map as M

class MPCModel(Enum):
    ROBOT = 1
    INVERTED_PENDULUM = 2

class MPCSampling(LocalPlanner):
    n_rollouts:int
    n_horizon:int
    speed:float
    dt:float
    yaw_range:Tuple[float, float]
    vehicle_length:float
    vehicle_width:float
    map:M.SoccerMap

    _map_resolution:float
    _bbox_x:np.ndarray
    _bbox_y:np.ndarray
    _bbox_map:np.ndarray
    _permissible:np.ndarray

    def __init__(self, lookahead_dist:float=0.6,
                 n_rollouts:int=24, n_horizon:int=20,
                 nominal_speed:float=0.5, dt:float=0.1,
                 yaw_range=(-0.34, 0.34),
                 vehicle_width:float=0.2, vehicle_length:float=0.33,
                 finish_threshold:float=0.2, exceed_threshold:float=4.0,
                 model=MPCModel.ROBOT,
                 collision_cost_weight=1000., tracking_cost_weight=1., pose_cost_weight=1.,
                 pose_cost_discount=0.9, pose_cost_state_weights=[1, 1, 1],
                 map_resolution:float=0.01):
        super(MPCSampling, self).__init__(lookahead_dist=lookahead_dist,
                                          finish_threshold=finish_threshold,
                                          exceed_threshold=exceed_threshold)

        self.n_rollouts = n_rollouts
        self.n_horizon = n_horizon
        self.speed = nominal_speed
        self.dt = dt

        if model is MPCModel.ROBOT:
            self.propagate_model = self._propagate_robot_model
        elif model is MPCModel.INVERTED_PENDULUM:
            self.propagate_model = self._propagate_inverted_pendulum_model
        self.yaw_range = yaw_range

        self.vehicle_length = vehicle_length
        self.vehicle_width = vehicle_width

        self._map_resolution = map_resolution
        self.map = M.SoccerMap.make_discrete_map(resolution=self._map_resolution)

        self._control_candidates = None
        self._poses = None
        self._bbox_px = None
        self._collision_weights = None
        self._tracking_weights = None
        self._pose_error_weights = None
        self.collision_cost_weight = collision_cost_weight
        self.tracking_cost_weight = tracking_cost_weight
        self.pose_cost_weight = pose_cost_weight
        self.pose_cost_state_weights = pose_cost_state_weights
        self.pose_cost_discount = pose_cost_discount

        # debug purposes
        self.prev_optimal_control = np.zeros((2, ))
        self.prev_X_dot_global = np.zeros((3, ))
        self.estimated_state = np.zeros((3, ))

        self.reset()

    def reset(self):
        """
        If you want to dynamically reset this instance, call this `reset` function after setting the parameters
        """
        super(MPCSampling, self).reset()
        # sample controls for each step [speed, yaw]
        self._control_candidates = np.zeros((self.n_rollouts, self.n_horizon, 2))
        self._control_candidates[:, :, 0] = self.speed
        self._control_candidates[:, :, 1] = np.tile(np.linspace(self.yaw_range[0], self.yaw_range[1], self.n_rollouts).reshape(self.n_rollouts, 1), (1, self.n_horizon)) # sweep each timestep

        # placeholders
        self._bbox_px = np.array([
            [ self.vehicle_length,  self.vehicle_width],
            [ self.vehicle_length, -self.vehicle_width],
            [-self.vehicle_length,  self.vehicle_width],
            [-self.vehicle_length, -self.vehicle_width]
        ]) / 2.0 / self._map_resolution # (4, 2)

        # cost weights
        self._collision_weights = np.arange(self.n_horizon + 1.0, 1, -1).reshape(1, self.n_horizon) # (1, n_horizon) single rollout over time horizon
        self._collision_weights = self.collision_cost_weight * np.tile(self._collision_weights, (self.n_rollouts, 1)) # (n_rollouts, n_horizon)
        pose_weight = np.ones((self.n_rollouts, 1)) # (n_rollouts, 1)
        pose_weight = self.pose_cost_state_weights * pose_weight # (n_rollouts, 3)
        self._pose_error_weights = self.pose_cost_weight * pose_weight # (n_rollouts, 3)
        self._tracking_weights = self.tracking_cost_weight

        # memory pre-allocation
        self._bbox_x = np.tile(self._bbox_px[:, 0], (self.n_rollouts * self.n_horizon, 1)) # (n_rollouts * n_horizon, 4:bbox_dim)
        self._bbox_y = np.tile(self._bbox_px[:, 1], (self.n_rollouts * self.n_horizon, 1)) # (n_rollouts * n_horizon, 4:bbox_dim)
        self._bbox_map = np.zeros((self.n_rollouts * self.n_horizon, 2, 4))  # (*, x/y, bbox_dim)
        self._permissible = np.zeros(self.n_rollouts * self.n_horizon, dtype=np.int64)

        self.optimal_control = np.zeros((3,))
        self.optimal_poses = np.zeros((self.n_horizon, 3))

    # Override
    def get_control(self, pose, ref_idx:int) -> np.ndarray:
        """
        :param pose: the current pose [x, y, yaw] of the robot (n_rollouts, n_horizon, 3)
        :param ref_idx: integer reference index in the global path
        :return: control [x_dot, y_dot, yaw_dot] in the robot's frame
        """

        # initialize rollouts (pose candidates)
        rollouts = np.zeros((self.n_rollouts, self.n_horizon, 3))
        rollouts[:, 0, :] = np.array(pose) # rollouts start at the current pose

        # simulate over time
        for tdx in range(self.n_horizon-1):
            X_dot = self.propagate_model(rollouts[:, tdx, :], self._control_candidates[:, tdx, :])
            rollouts[:, tdx + 1, :] = rollouts[:, tdx, :] + X_dot * self.dt

        # calculate costs = collision + error + pose difference b/w the current and the final step
        collisions = self.check_collisions(rollouts.reshape(-1, 3)).reshape(self.n_rollouts, self.n_horizon) # (n_rollouts, 3)
        collision_cost = np.sum(collisions * self._collision_weights, axis=1) # (n_rollouts, )

        tracking_error = rollouts[:, self.n_horizon - 1, :2] - self._path[ref_idx, :2]   # (n_rollouts, 2), 2D
        tracking_cost = np.linalg.norm(tracking_error, axis=1) * self._tracking_weights  # (n_rollouts,)

        # pose_error = rollouts[:, self.n_horizon - 1, :] - self._path_with_yaw[-1, :] # (n_rollouts, 3)
        pose_error = rollouts[:, self.n_horizon - 1, :] - self._path[-1, :]  # (n_rollouts, 3)
        pose_error = self._pose_error_weights * pose_error * self.pose_cost_discount ** (self._path.shape[0] - ref_idx - 1)
        final_pose_cost = np.linalg.norm(pose_error, axis=1) # (n_rollouts, )

        optimal_control_idx = np.argmin(collision_cost + tracking_cost + final_pose_cost)
        optimal_control = self._control_candidates[optimal_control_idx, :, :] # (n_rollouts, n_horizon, 2) # find the control trajectory with the minimum cost
        optimal_control = optimal_control[0, :] # (2, ) receiding horizon control taking only one step
        self.optimal_poses[:, :3] = rollouts[optimal_control_idx, :, :] # (n_horizon, 2)

        # transform to a command set that our robot prefers, represented in the robot's local frame
        X_dot = self.propagate_model(rollouts[None, 0, 0, :], optimal_control[None, :]).T # transpose exists...... (3, n_rollouts)
        self.optimal_control[:] = self.speed, 0, X_dot[2, 0] # X_dot in robot frame

        # debug purposes
        self.prev_optimal_control = optimal_control
        self.prev_X_dot_global = X_dot
        self.estimated_state = rollouts[optimal_control_idx, 1, :]

        return self.optimal_control

    def _propagate_robot_model(self, rollouts:np.ndarray, control):
        """
        based on the rear wheel of the robot model
        :param rollouts: (n_rollouts, 3) - [[x, y, z], ]: current n_rollouts poses of the robot
        :param control:  (n_rollouts, 2) - [[speed, steering_angle], ]: current controls to step forward
        :param L: the length of wheel base
        :return: state_change_rate: (3, n_rollouts) - [x_d ...; y_d ...; yaw_d ...]
        """
        speed = control[:, 0] # (n_rollouts, ) first col of all rows

        X_dot = np.array([
            np.cos(rollouts[:, 2]), 
            np.sin(rollouts[:, 2]),
            np.tan(control[:, 1]) / self.vehicle_length
        ]) # (3, n_rollouts)

        return (speed * X_dot).T # (n_rollouts, 3)

    def _propagate_inverted_pendulum_model(self, rollouts:np.ndarray, control):
        """
        based on the inverted pendulum model
        :param rollouts: (n_rollouts, 3) - [[x, y, z], ]: current n_rollouts poses of the robot
        :param control:  (n_rollouts, 2) - [[speed, steering_angle], ]: current controls to step forward
        :return: state_change_rate: (3, n_rollouts) - [x_d ...; y_d ...; yaw_d ...]
        """
        X_dot = np.tile(np.array([[0.], [0.], [0.]]), (1, rollouts.shape[0]))
        raise NotImplementedError  # TODO: RC-158
        return X_dot

    def check_collisions(self, rollouts) -> np.ndarray:
        """
        check_collisions_in_map is a collision checker that determines whether a set of K * T poses
            are in collision with occupied pixels on the map.
        :param rollouts: (n_rollouts * n_horizon, 3) - poses to check for collisions on
        :return collisions: (n_rollouts * n_horizon, 1) - float vector where 1.0 == collision, 0.0 == no collision
            for the input pose with corresponding index.
        """
        # bbox corners before rotation
        c, s = np.cos(rollouts[:, 2, None]), np.sin(rollouts[:, 2, None]) # (n_rollouts * n_horizon, 1) respectively

        # rotate bounding box edges to the current forward direction
        self._bbox_map[:] = 0.
        self._bbox_map[:, 0, :] = (self._bbox_x * c - self._bbox_y * s) + np.tile(rollouts[:, 0, None], 4) # (n_rollouts * n_horizon, 4)
        self._bbox_map[:, 1, :] = (self._bbox_x * s + self._bbox_y * s) + np.tile(rollouts[:, 1, None], 4)  # (n_rollouts * n_horizon, 4)
        bbox_int = self._bbox_map.astype(np.int16)

        # check collisions with static obstacles in the map
        self._permissible[:] = 0
        for bdx in range(self._bbox_px.shape[0]): # check if there is static obstacle at bbox corners
            self._permissible[:] = np.logical_or(self._permissible, self.map[bbox_int[:, 1, bdx], bbox_int[:, 0, bdx]])[:]

        # check collision with dynamic obstacles
        # expand pose (in obs-wise) and obstacle (pose-wise) arrays to take diff in every case
        with self._obstacles_lock:
            pose_exp = np.tile(rollouts[:, None, :2], (1, self._obstacles.shape[0], 1)) # (n_rollouts, n_obstacles, 2)
            obs_exp = np.tile(self._obstacles[None, :, :2], (rollouts.shape[0], 1, 1)) # (n_rollouts, n_obstacles, 2)
        distance = np.linalg.norm(pose_exp - obs_exp, axis=2) # (n_rollouts, n_obstacles)
        in_collision = np.any(distance < 1.0, axis=1) # (n_rollouts, ) check
        self._permissible[:] = np.logical_or(self._permissible, in_collision)[:]

        return self._permissible.astype(np.float64)