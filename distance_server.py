
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import math

class DistanceServer(Node):
    def __init__(self):
        super().__init__('distance_server')
        
        # Subscriber to LaserScan topic
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Publisher for total distance
        self.distance_publisher = self.create_publisher(Float64, '/total_distance', 10)

        # Initialize variables
        self.previous_x = None
        self.previous_y = None
        self.total_distance = 0.0

    def lidar_callback(self, msg):
        # Assume front-facing LiDAR with angle_min = 0 and angle_increment for simplicity
        angle = msg.angle_min
        
        # Use the nearest point in the front direction (angle close to 0)
        front_distance = None
        for i, distance in enumerate(msg.ranges):
            if math.isfinite(distance):
                if front_distance is None or distance < front_distance:
                    front_distance = distance
                    angle = msg.angle_min + i * msg.angle_increment

        if front_distance is not None:
            # Calculate current position assuming flat 2D plane
            current_x = front_distance * math.cos(angle)
            current_y = front_distance * math.sin(angle)

            # Calculate distance moved
            if self.previous_x is not None and self.previous_y is not None:
                dx = current_x - self.previous_x
                dy = current_y - self.previous_y
                self.total_distance += math.sqrt(dx**2 + dy**2)

            # Update previous coordinates
            self.previous_x = current_x
            self.previous_y = current_y

            # Publish the total distance
            distance_msg = Float64()
            distance_msg.data = self.total_distance
            self.distance_publisher.publish(distance_msg)

            self.get_logger().info(f'Total Distance: {self.total_distance:.2f} meters')

def main(args=None):
    rclpy.init(args=args)
    
    distance_server = DistanceServer()

    try:
        rclpy.spin(distance_server)
    except KeyboardInterrupt:
        pass

    distance_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
