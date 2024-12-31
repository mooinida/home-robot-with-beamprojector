
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math

class StopHandler(Node):
    def __init__(self):
        super().__init__('stop_handler')

        # Subscriber to LaserScan topic
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Publisher for obstacle handling
        self.obstacle_publisher = self.create_publisher(String, '/obstacle_status', 10)

        self.stop_threshold = 0.5  # Distance threshold in meters

    def lidar_callback(self, msg):
        # Assume a 180-degree front-facing LiDAR
        min_distance = min(msg.ranges)
        
        if min_distance < self.stop_threshold:
            # Obstacle detected
            self.publish_obstacle_status("Obstacle Detected")
            self.handle_obstacle()
        else:
            # Clear path
            self.publish_obstacle_status("Path Clear")

    def publish_obstacle_status(self, status):
        status_msg = String()
        status_msg.data = status
        self.obstacle_publisher.publish(status_msg)
        self.get_logger().info(f'Obstacle Status: {status}')

    def handle_obstacle(self):
        self.get_logger().info("Handling obstacle: Stopping and deciding next action.")
        # Placeholder for actual obstacle handling logic
        # For example: stop motors, plan a new path, or notify another node

def main(args=None):
    rclpy.init(args=args)

    stop_handler = StopHandler()

    try:
        rclpy.spin(stop_handler)
    except KeyboardInterrupt:
        pass

    stop_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
