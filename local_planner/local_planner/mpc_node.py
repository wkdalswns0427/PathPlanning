#!/usr/bin/env python3
from utils.imports import *
from utils.utils import *
from modules.mpc import MPCSampling, MPCModel
from modules.local_planner import LocalPlanner
# from gole.algorithms.mpc import MPCSampling
from pp_interfaces.action import LocalPath
from pp_interfaces.msg import Path
from rclpy import logging

sys.path.append('/home/mjc/dev/proj/gole_core/gole/utils')
import shared_memory
controllers = {'MPC': MPCSampling}

class MPCNode(Node):
    robot_name:str
    prefix:str

    controller:LocalPlanner = None
    freq:float = 50
    config:dict = {}
    _running = Event()
    _ready = Event()
    _path_event = Event()
    _reset_lock = Lock()

    _current_pose: np.ndarray = np.zeros((3, ))
    _goal_position: np.ndarray = np.zeros((2,))

    def __init__(self):
        super().__init__('mpc_node')
        self.trajectory_path = "/home/mjc/dev/proj/gole_core_ros/gole/gole_pathschedule/gole_pathschedule/trajectory/"
        self.shm = shared_memory.SharedMemory("gole-001")
        self.pose = np.zeros(3)
        self.MPCPlanner = MPCSampling()
        self.reference = np.load(self.trajectory_path+"reference.npy")
        self.set_path(self.reference)

        # action name : localpath_action
        # action_type : gole_interfaces/action/LocalPath
        # ros2 action send_goal <action_name> <action_type> <values> --feedback
        self._action_server = ActionServer(
            self,
            LocalPath,
            'localpath_action',
            self.actionCallback
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 100)

    # Action Server return in response to request
    def actionCallback(self, goal_handle):
        self.get_logger().info('executing goal')

        self.odom_state = self.shm.get("OdomState")
        self.pose = self.odom_state.p_ob[:]

        self.MAX_IDX = len(self.reference[0]) - 1

        self.run()

        result = LocalPath.Result()
        result.status = True

        goal_handle.succeed()
        return result

    def publish_cmdvel(self, control):
        payload = Twist()
        payload.linear.x = control[0]; payload.angular.z = control[2]
        self.publisher_.publish(payload)
        self.get_logger().info(f"cmd_vel -> lin x : {payload.linear.x}, ang_z = {payload.angular.z}")

    def run(self):
        # rate = rclpy.Rate(self.freq)
        self.is_ready = True
        self._loop_rate = self.create_rate(self.freq, self.get_clock())
        while rclpy.ok():
            if not self._path_event.is_set():
                self.get_logger().info("Waiting for a new global path to be set...")
                self._path_event.wait()
                continue

            self._reset_lock.acquire()

            if not self.is_ready:
                self.get_logger().info("Current pose is not set")
                self._ready.wait()
            else:
                self.get_logger().info("Generating Path")
                ref_idx, ref_pose = self.MPCPlanner.get_ref_pose(self.pose)
                error = self.MPCPlanner.get_error(self.pose)
                # publish ref_pose and error
                control = self.MPCPlanner.get_control(self._current_pose, ref_idx)
                planned = self.MPCPlanner.optimal_poses

                self.publish_cmdvel(control)

                if self.MPCPlanner.is_path_completed(self._current_pose, error):
                    self.get_logger().info("The path is completed!!!")
                    self._path_event.clear()
            self._reset_lock.release()
            self._loop_rate.sleep()

    def set_path(self, path): 
        self.get_logger().info("Setting a reference path to the local planner...")
        self._reset_lock.acquire()
        self.MPCPlanner.set_path(path)
        self._path_event.set()
        self._reset_lock.release()

    def set_goal_position(self, data):
        self.get_logger().info(f"update_goal_position: {data}")
        if isinstance(data, Point):
            self._goal_position = np.array([data.x, data.y])
        elif isinstance(data, Pose):
            self._goal_position = np.array([data.position.x, data.position.y])
        else:
            self.get_logger().info(f"Wrong goal position/pose type is given: Need to be Point or Pose")
            return

    #============
    # Initialize
    #============
    def reset(self):
        self.get_logger().info("Resetting Local Planner...")
        if self.MPCPlanner is None:
            self.MPCPlanner = MPCSampling()
        else:
            self.MPCPlanner.reset()
        self._reset_lock.acquire()
        self._current_pose[:] = 0.0, 0.0, 0.0
        self.reference = np.load(self.trajectory_path+"reference.npy")
        self.set_path(self.reference)
        self._path_event.clear()
        self._reset_lock.release()
        return []


def main(args=None):
    print('Initiaing MPC')
    rclpy.init(args=args)
    mpc_node = MPCNode()
    rclpy.spin(mpc_node)

    mpc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()