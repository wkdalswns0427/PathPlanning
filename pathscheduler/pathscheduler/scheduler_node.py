from gole_pathschedule.utils.imports import *
from gole_interfaces.action import GlobalPath
from gole_interfaces.action import LocalPath

class ScheduleSubscriber(Node):

    def __init__(self):
        super().__init__('gole_pathschedule')
        self.global_flag, self.local_flag = 0, 0
        self.trajectory_path = "./trajectory/"
        self.subscription = self.create_subscription(
            Bool,
            '/PathSchedule',
            self.schedule_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.globalpath_action_client = ActionClient(self, GlobalPath, 'globalpath_action')
        self.localpath_action_client = ActionClient(self, LocalPath, 'localpath_action')

    def schedule_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
        if msg.data:
            self.globalpath_goal()
        elif not msg.data:
            self.localpath_goal()
    
    def globalpath_goal(self):
        self.global_flag = 1
        global_goal = GlobalPath.Goal()
        global_goal.order = True
        self.globalpath_action_client.wait_for_server()
        self._global_goal_future = self.globalpath_action_client.send_goal_async(global_goal)
        self._global_goal_future.add_done_callback(self.goal_response_callback)
        # return self.globalpath_action_client.send_goal_async(global_goal)

    def localpath_goal(self):
        self.local_flag = 1
        local_goal = LocalPath.Goal()
        local_goal.order = True
        
        self.localpath_action_client.wait_for_server()
        self._local_goal_future = self.localpath_action_client.send_goal_async(local_goal)
        self._local_goal_future.add_done_callback(self.goal_response_callback)
        # return self.localpath_action_client.send_goal_async(local_goal)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if self.global_flag == 1:
            temp = []
            for i in reversed(range(len(result.sequence_x))):
                tempx = round(result.sequence_x[i],3)
                tempy = round(result.sequence_y[i],3)
                if i > 0:
                    tempx_n = round(result.sequence_x[i-1],3)
                    tempy_n = round(result.sequence_y[i-1],3)
                    tempw = round(np.arctan2(tempx-tempx_n,tempy-tempy_n),3)
                else:
                    tempw = 0

                temp.append([tempx, tempy, tempw])
            self.get_logger().info(f'Result: {temp}')
            np.save(self.trajectory_path+"reference.npy", temp)
            self.global_flag = 0
        elif self.local_flag == 1:
            self.get_logger().info(f'Result: {result.status}')
            self.local_flag = 0


def main(args=None):
    print('Initiaing Scheduler')
    rclpy.init(args=args)
    schedule_subscriber = ScheduleSubscriber()
    rclpy.spin(schedule_subscriber)
    schedule_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
