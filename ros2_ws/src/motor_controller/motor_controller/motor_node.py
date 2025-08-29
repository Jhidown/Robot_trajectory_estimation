import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from std_msgs.msg import String
import time


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Publisher CAN
        self.publisher = self.create_publisher(Frame, 'can_tx', 10)

        # Subscription to text commands
        self.subscription = self.create_subscription(
            String,
            'motor_cmd',
            self.cmd_callback,
            10
        )

        self.get_logger().info("MotorController ready (listening on /motor_cmd: 'SPEED <right> <left>")

    def cmd_callback(self, msg: String):
        parts = msg.data.strip().split()
        if len(parts) != 3 or parts[0].upper() != "SPEED":
            self.get_logger().warn(f"Message ignored: {msg.data}")
            return

        try:
            speed_right = int(parts[1])
            speed_left = int(parts[2])
        except ValueError:
            self.get_logger().error(f"Invalid values: {parts[1:]} (integers expected)")
            return

        # Envoi des commandes
        self.send_motor_command(0x601, speed_right)
        time.sleep(0.025)
        self.send_motor_command(0x602, speed_left)

    def send_motor_command(self, can_id, speed):
    	data = bytearray([
        	0x23,       # SDO command (write 4 bytes)
        	0xFF, 0x60, 0x00  # Index 0x60FF:00
    	])
    	data.extend(int(speed).to_bytes(4, byteorder='little', signed=True))

    	msg = Frame()
    	msg.id = can_id
    	msg.is_extended = False
    	msg.is_rtr = False
    	msg.is_error = False
    	msg.dlc = 8
    	msg.data = list(data)

    	self.publisher.publish(msg)
    	self.get_logger().info(f"[CMD] {can_id:03X} TargetVelocity={speed}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

