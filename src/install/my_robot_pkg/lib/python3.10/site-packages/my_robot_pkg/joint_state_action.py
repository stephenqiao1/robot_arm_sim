import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointStateToGazeboBridge(Node):
    def __init__(self):
        super().__init__('joint_state_to_gazebo_bridge')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        self.subscription # prevent unused variable warning
        
        # Initialize publisher for joint_trajectory_controller
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info('JointStateToGazeboBridge Node started')
        
    def listener_callback(self, msg):
        self.get_logger().info('Received a message on /joint_states')
        
        # Create a joint trajectory message
        trajectory = JointTrajectory()
        trajectory.header = Header()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = msg.name
        
        self.get_logger().info(f'Joint names: {trajectory.joint_names}')
        
        point = JointTrajectoryPoint()
        point.positions = msg.position
        point.time_from_start.sec = 1 # Set a suitable duration for the movement
        
        trajectory.points.append(point)
        
        self.get_logger().info(f'Publishing trajectory to /my_robot_pkg/joint_trajectory_controller/command')
        
        # Publish the trajectory
        self.trajectory_publisher.publish(trajectory)
        
def main(args=None):
    rclpy.init(args=args)
    bridge = JointStateToGazeboBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()