#!/usr/bin/env python3
from local_planner.modules.mpc import MPCSampling
# from gole.algorithms.mpc import MPCSampling
from local_planner.utils.imports import *
from pp_interfaces.action import LocalPath

class MPCNode(Node):

    def __init__(self):
        super().__init__('mpc_node')
        # action name : localpath_action
        # action_type : gole_interfaces/action/LocalPath
        # ros2 action send_goal <action_name> <action_type> <values> --feedback
        MPCPlanner = MPCSampling()
        self._action_server = ActionServer(
            self,
            LocalPath,
            'localpath_action',
            self.actionCallback
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd__vel', 10)

    # Action Server return in response to request
    def actionCallback(self, goal_handle):
        self.get_logger().info('executing goal')
        feedback_msg = LocalPath.Feedback()

        """
        MPCSampling.get_control()
        :param pose: the current pose [x, y, yaw] of the robot (n_rollouts, n_horizon, 3)
        :param ref_idx: integer reference index in the global path
        :return: control [x_dot, y_dot, yaw_dot] in the robot's frame
        """
        self.control = MPCSampling.get_control()
        self.publish__cmdvel(self.control)
        result = LocalPath.Result()
        result.status = True

        goal_handle.succeed()
        return result

    def publish__cmdvel(self, control):
        payload = Twist()
        payload.linear.x = control[0]; payload.angular.z = control[2]
        self.publisher_.publish(payload)
        self.get_logger().info(f"cmd_vel -> lin x : {payload.linear.x}, ang_z = {payload.angular.z}")


def main(args=None):
    print('Initiaing MPC')
    rclpy.init(args=args)
    mpc_node = MPCNode()
    rclpy.spin(mpc_node)

    mpc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()