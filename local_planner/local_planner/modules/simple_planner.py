from local_planner.utils.imports import *
class PDController:

    def __init__(self,
                 Kp,
                 Kd,
                 ctrl_max):
        self.Kp = Kp
        self.Kd = Kd
        self.ctrl_max = ctrl_max
        self.error_prev = np.zeros( np.shape(Kd) )


    def reset(self, Kp, ctrl_max):
        self.Kp = Kp
        self.ctrl_max = ctrl_max


    def set_p_gain(self, Kp):
        self.Kp = Kp

    def set_d_gain(self, Kd):
        self.Kd = Kd


    def get_control(self, error, dt=0.01):
        output = self.Kp * error + self.Kd * (error - self.error_prev) / dt
        return np.clip(output, -self.ctrl_max, self.ctrl_max)


class LowpassFilter:
    def __init__(self,
                 alpha: float,
                 x0: float = np.zeros(1)):
        # Decay factor
        self._alpha = alpha

        # Current filter state
        self._x = x0

    def set_state(self, x0):
        self._x = x0

    def update(self, x):
        self._x = self._alpha * x + (1. - self._alpha) * self._x

        # Make any measurements close to zero equal 0.0
        # self._x[np.abs(self._x) < 1e-8] = 0.0
        return self._x


class Phases(enum.IntEnum):
    YAW_TO_XY     = 0
    FORWARD_TO_XY = 1
    YAW_TO_GOAL   = 2


class BallFollower:
    def __init__(self,
                 Kp_vel,
                 Kp_yaw,
                 Kp_position = np.array([0.5, 0.1]),
                 v_bb_max = np.array([0.3, 0.15]),
                 wyaw_bb_max = 0.6,
                 close_to_ball_tol = 0.95,
                 yaw_tol = 0.3,
                 ):
        self.p_wb = np.array([0., 0.])
        self.yaw_wb = 0.

        self.p_wb_desired = np.array([0., 0.])
        self.yaw_wb_desired = 0.0
        
        self.v_bb_max = v_bb_max
        self.w_yaw_bb_max = wyaw_bb_max

        self.yaw_tol = yaw_tol
        self.close_to_ball_tol = close_to_ball_tol
        self._v_bb_lowpass = LowpassFilter(0.04, np.zeros(2))
        self._wyaw_bb_lowpass = LowpassFilter(0.1, 0.0)

        self.err_dist = 0.0


        self.Kp_vel = Kp_vel
        self.Kd_vel = 0.0
        self.Kp_yaw = Kp_yaw
        self.Kd_yaw = 0.0
        self.Kp_position = Kp_position
        self.Kd_position = np.array([0.0, 0.0])


        self._v_bb_controller = PDController(self.Kp_vel, self.Kd_vel, self.v_bb_max[0])
        self._wyaw_bb_controller = PDController(self.Kp_yaw, self.Kd_yaw, self.w_yaw_bb_max)
        
        self._p_bb_controller = PDController(self.Kp_position,
                                             self.Kd_position,
                                             self.v_bb_max)


    def set_current_position_yaw(self, p_wb, yaw_wb):
        self.p_wb = p_wb
        self.yaw_wb = yaw_wb

    def set_current_pose(self, pose_wb):
        self.p_wb = pose_wb[:2]
        self.yaw_wb = pose_wb[2]

    def set_desired_position_yaw(self, p_wb, yaw_wb):
        self.p_wb_desired = p_wb
        self.yaw_wb_desired = yaw_wb

    def set_desired_pose(self, pose_wb_desired):
        self.p_wb_desired = np.reshape(pose_wb_desired, (3,))[:2]
        self.yaw_wb_desired = np.reshape(pose_wb_desired, (3,))[2]


    def get_control(self):
        """
        
        Behavior:
            P(ID) yaw rate around yaw error
            Use distance to ball as the main switch:
                When the robot is far away from the ball, change yaw setpoint to face the ball
                Only want to move when the yaw error (pointing the right direction)
                    When yaw error is asmall -> P(ID) forward (x) velocity around distance to ball
                    When yaw eror is large -> no body velocity (focus on turning toward ball)
                When the robot is close to the ball, change yaw setpoint to face goal
                Having only forward (x) velocity will not be sufficient when turning -> Omni direction P(ID) around position to ball


        """

        ### Get body frame ball position, yaw
        # Convert from world frame into body frame. Body in the world frame -> rotating world into body is R.T @ x
        R_wb = np.array([[ np.cos(self.yaw_wb), -np.sin(self.yaw_wb)],
                         [ np.sin(self.yaw_wb),  np.cos(self.yaw_wb)]])

        # Get the ball position and yaw in body frame
        p_bball = R_wb.T @ (self.p_wb_desired - self.p_wb)
        yaw_bball = np.arctan2( p_bball[1], p_bball[0])


        ### Get distance to ball and world goal yaw
        # Get the distance to the ball
        dist_to_ball = lin.norm(p_bball)
        self.err_dist = dist_to_ball

        # Get the yaw to the goal
        yaw_bgoal = self.yaw_wb_desired - self.yaw_wb


        ### Determine yaw setpoint and velocity behavior
        # if the distance to ball is large -> yaw set point = ball yaw in body frame
        if dist_to_ball > self.close_to_ball_tol:
            error_yaw = yaw_bball


            # if yaw error is large -> slow down to focus on turning
            if np.abs(error_yaw) > self.yaw_tol:
                v_bb_command = np.array([0.0, 0.0])

            # if yaw error is small -> P(ID) to output forward (x) velocity
            else:
                v_bb_x_command = self._v_bb_controller.get_control(dist_to_ball)
                # No y control
                v_bb_command = np.array([v_bb_x_command, 0.0])

        # if distance to ball is small -> yaw set point = goal yaw in body frame
        else:
            error_yaw = wrap_between_pi_and_neg_pi(yaw_bgoal)
            v_bb_command = self._p_bb_controller.get_control(p_bball)

        ### Process yaw control
        # P(ID) to get to yaw setpoint
        wyaw_bb_command = self._wyaw_bb_controller.get_control(error_yaw)

        ### Low pass control
        # Lowpass signals to smooth out changes to yaw setpoint
        v_bb = self._v_bb_lowpass.update(v_bb_command)
        wyaw_bb = self._wyaw_bb_lowpass.update(wyaw_bb_command)

        return v_bb, wyaw_bb, error_yaw

def wrap_between_pi_and_neg_pi(angle):
    return (angle + np.pi) % (2. * np.pi) - np.pi




class SingleBody2D:
    def __init__(self,
                 p_wb: np.ndarray, 
                 yaw_wb: float,
                 ):
        self.p_wb = p_wb
        self.yaw_wb = yaw_wb

    def simulate(self, v_bb, wyaw_bb, dt):
        # Rotation matrix of world to body frame
        R_wb = np.array([[ np.cos(self.yaw_wb), -np.sin(self.yaw_wb)],
                         [ np.sin(self.yaw_wb),  np.cos(self.yaw_wb)]])

        # Convert body frame commands to world frame state changes
        self.p_wb += (R_wb @ v_bb) * dt
        self.yaw_wb += wyaw_bb * dt

        return self.p_wb, self.yaw_wb


if __name__ == "__main__":
    import numpy as np
    from matplotlib import pyplot as plt

    # Initial pose of the body in the world frame
    p_wb = np.array([5.0, 1.0])
    v_bb = np.array([0., 0.])
    yaw_wb = 0
    wyaw_wb = 0.

    ps_wb = np.array([*p_wb, yaw_wb])
    vs_bb = np.array([*v_bb, wyaw_wb])


    # Construct a body with the initial pose
    body = SingleBody2D(p_wb, yaw_wb)


    # Desired pose of the body in the world frame
    p_wb_desired = np.array([8.0, 4.0])
    yaw_wb_desired = .5
    ps_wb_desired = np.array([*p_wb_desired, yaw_wb_desired])

    R_wb = np.array([[ np.cos(yaw_wb),-np.sin(yaw_wb)],
                     [ np.sin(yaw_wb), np.cos(yaw_wb)]])

    # Get the ball position and yaw in body frame
    p_bball = R_wb.T @ (p_wb_desired - p_wb)
    yaw_bball = np.arctan2( p_bball[1], p_bball[0])
    yaw_bgoal = yaw_wb_desired - yaw_wb


    planner = BallFollower(0.5, 2.0)


    #@ Create P controller for body frame acceleration with body frame position feedback
    # Make gains fast for x and yaw control, but limit y control
    Kp = np.array([0.1, 0.02, 0.1]) # [m/s^2 / m] Each m of error leads to Kp m/s^2 acceleration
    as_bb_max = np.array([0.01, 0.005, 0.02]) # [m/s^2] Clip acceleration to slowly ramp body frame velocity
    vs_bb_max = np.array([0.25, 0.02, 0.5]) # [m/s] Clip velocity to limit rate of movement
    # controller = PDController(Kp, as_bb_max)



    # Create the time points to simluate time at
    dt = 0.01
    t_sim_arr = np.arange(0, 30, dt)
    nt_sim_arr = len(t_sim_arr)


    ps_wb_store = np.zeros( (nt_sim_arr, 3) )
    vs_wb_store = np.zeros( (nt_sim_arr, 3) )
    err_store = np.zeros( (nt_sim_arr, 4) )

    for k, tk in enumerate(t_sim_arr):


        R_wb = np.array([[ np.cos(yaw_wb), -np.sin(yaw_wb)],
                         [ np.sin(yaw_wb),  np.cos(yaw_wb)]])
        p_bb = R_wb @ p_wb
        p_bb_desired = R_wb @ p_wb_desired

        ps_bb = np.array([*p_bb, yaw_wb])
        ps_bb_desired = np.array([*p_bb_desired, yaw_wb_desired])



        planner.set_current_position_yaw(p_wb, yaw_wb)
        planner.set_desired_position_yaw(p_wb_desired, yaw_wb_desired)
        v_bb, wyaw_wb, err_yaw = planner.get_control()


        p_wb, yaw_wb = body.simulate(v_bb, wyaw_wb, dt)
        ps_wb_store[k, :] = np.array([*p_wb, yaw_wb])
        err_store[k, :] = np.array([*(p_wb_desired - p_wb), err_yaw, planner.err_dist])
        vs_wb_store[k, :] = np.array([*v_bb, wyaw_wb])

    #     ps_wb = np.array([*p_wb, yaw_wb])


    # planner = PDController(p.array([0.5, 0.1, 5.0]))
    # planner.get_control()


    # fig, ax = plt.subplots()
    # ax.plot(t_sim_arr, ps_wb_store[:,0], "b", t_sim_arr, ps_wb_store[:,1], "g")
    # ax1 = ax.twinx()
    # ax1.plot(t_sim_arr, ps_wb_store[:,2], "r--")


    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2,2)
    ax1.plot(t_sim_arr, err_store[:,0], "b", t_sim_arr, err_store[:,1], "g", t_sim_arr, err_store[:,3], "k")
    ax1.plot(t_sim_arr, np.zeros( len(t_sim_arr) ), "r--")
    ax1.legend(["x error", "y error", "norm error", "zero"])

    ax2.plot(t_sim_arr, ps_wb_store[:,2], "k")
    ax2.plot(t_sim_arr, yaw_bball *np.ones( len(t_sim_arr) ), "g--", t_sim_arr, yaw_bgoal *np.ones( len(t_sim_arr) ), "r--")
    ax2.legend(["yaw command", "yaw ball", "yaw goal"])


    ax3.plot(t_sim_arr, vs_wb_store[:,0], "b", t_sim_arr, vs_wb_store[:,1], "g")
    ax3.plot(t_sim_arr, np.zeros( len(t_sim_arr) ), "r--")
    ax3.legend(["x vel command", "y vel command", "zero"])

    ax4.plot(t_sim_arr, vs_wb_store[:,2], "k")
    ax4.plot(t_sim_arr, np.zeros( len(t_sim_arr) ), "r--")
    ax4.legend(["yaw rate command", "zero"])
    plt.show()

    # plt.show()
