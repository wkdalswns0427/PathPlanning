#!/usr/bin/env python3
from local_planner.utils.imports import *
from local_planner.modules.pure_pursuit import *
from pp_interfaces.action import LocalPath

class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('mpc_node')
        # action name : localpath_action
        # action_type : gole_interfaces/action/LocalPath
        # ros2 action send_goal <action_name> <action_type> <values> --feedback
        self.PurePursuitPlanner = PurePursuit()
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

        # CALL MAP MANAGER -> get MAP
        # CALL GLOBAL PATH -> get Global Path(reference trajectory)
        #dummy
        ref = []
        ref.append([np.arange(0, 50, 0.5)])
        ref.append([math.sin(ix / 5.0) * ix / 2.0 for ix in cx])
        """
        pure pursuit algorithm usage
        cx, cy : target path [np.array]
        initial_pos : [x,y, yaw, v]
        """
        self.control = self.PurePursuitPlanner.pure_pursuit(ref[0], ref[1], [0.0,0.0,0.0,0.0])
        self.publish_cmdvel(self.control)

        # action result
        result = LocalPath.Result()
        result.status = True

        goal_handle.succeed()
        return result

    def publish_cmdvel(self, control):
        payload = Twist()
        payload.linear.x = control[0]; payload.angular.z = control[2]
        self.publisher_.publish(payload)
        self.get_logger().info(f"cmd_vel -> lin x : {payload.linear.x}, ang_z = {payload.angular.z}")


def main(args=None):
    print('Initiaing Pure Pursuit')
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuitNode()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()