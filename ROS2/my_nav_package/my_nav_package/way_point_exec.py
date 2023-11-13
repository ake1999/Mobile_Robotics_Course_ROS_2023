import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class WaypointExecutor(Node):
    def __init__(self):
        super().__init__('waypoint_executor')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.result_future = None
        self.waypoints = [
            self.create_waypoint(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            self.create_waypoint(1.0, -1.8, 0.0, 0.0, 0.0, 0.0, 1.0),
            self.create_waypoint(-2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        ]

    def create_waypoint(self, x, y, z, qx, qy, qz, qw):
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'map'
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = z
        waypoint.pose.orientation.x = qx
        waypoint.pose.orientation.y = qy
        waypoint.pose.orientation.z = qz
        waypoint.pose.orientation.w = qw
        return waypoint

    def send_waypoint(self, waypoint):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))

    def run(self):
        for waypoint in self.waypoints:
            self.send_waypoint(waypoint)
            while not self.result_future or not self.result_future.done():
                rclpy.spin_once(self)

            result = self.result_future.result().result
            self.get_logger().info(f'Reached waypoint with result: {result}')
            self.result_future = None

        self.get_logger().info('All waypoints have been visited.')

def main(args=None):
    rclpy.init(args=args)
    waypoint_executor = WaypointExecutor()

    try:
        waypoint_executor.run()
    except KeyboardInterrupt:
        pass

    #waypoint_executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
