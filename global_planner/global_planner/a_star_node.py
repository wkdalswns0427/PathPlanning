#!/usr/bin/env python3
from global_planner.utils.imports import *
from global_planner.modules.a_star_planner import *
# from gole.algorithms.gole_astar import *
from rclpy.action import ActionServer
from pp_interfaces.action import GlobalPath

class AStarNode(Node):

    def __init__(self):
        super().__init__('a_star_node')
        # action name : globalpath_action
        # action_type : pp_interfaces/action/Globalpath
        # ros2 action send_goal <action_name> <action_type> <values> --feedback
        self._action_server = ActionServer(
            self,
            GlobalPath,
            'globalpath_action',
            self.actionCallback
        )


    # Action Server return in response to request
    def actionCallback(self, goal_handle):
        self.get_logger().info('executing goal')
        feedback_msg = GlobalPath.Feedback()

        # {algorithm}_planner_go(parameter_path)
        rx, ry, res = astar_planner_go(os.path.join(os.environ["GOLE_CORE_CONFIG"], "global_pp.yaml"))
        result = GlobalPath.Result()
        result.sequence_x = rx
        result.sequence_y = ry

        goal_handle.succeed()
        return result


def main(args=None):
    print('Initiaing A_STAR_PLANNER')
    rclpy.init(args=args)
    a_star_node = AStarNode()
    rclpy.spin(a_star_node)

    a_star_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
