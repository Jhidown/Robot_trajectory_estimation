import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import copy

class LidarReaderNode(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
    	angle_min = msg.angle_min   #radian
    	angle_increment = msg.angle_increment
    	ranges = msg.ranges
    	
    	
    	angle_start = math.radians(-20)
    	angle_end = math.radians(20)
    	
    	#corresponding indices in the array
    	index_start = int((angle_start - angle_min) / angle_increment)
    	index_end = int((angle_end - angle_min) /  angle_increment)
    	
    	#clamp within the limits of the array
    	index_start = max(0,index_start)
    	index_end = min(len(ranges) - 1, index_end)
    	
    	#extract data
    	filtered_ranges = ranges[index_start:index_end +1]
    	
    	#display the minimum distance in this field of view
    	filtered_ranges = [r for r in filtered_ranges if not math.isinf(r)]
    	if filtered_ranges:
    		min_dist = min(filtered_ranges)
    		self.get_logger().info(f"Obstacle between -20° and 20° at {min_dist:.2f}m")
    		
    	else :
    		self.get_logger().info("No obstacle between -20° and 20°")


def main(args=None):
    rclpy.init(args=args)
    node = LidarReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

