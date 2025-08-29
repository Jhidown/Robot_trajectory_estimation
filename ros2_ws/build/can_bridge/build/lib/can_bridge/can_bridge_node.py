import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
import serial
import time


def build_framework(can_id, data):
    dlc = len(data)
    if dlc > 8:
        raise ValueError("maximum 8 bytes of data")
    control = 0xC0 | dlc
    frame = bytearray([0xAA, control, can_id & 0xFF, (can_id >> 8) & 0xFF])
    frame.extend(data)
    frame.append(0x55)
    return frame


def read_work(buffer):
    while True:
        # Search for start marker 0xAA
        start = buffer.find(b'\xAA')
        if start == -1:
            buffer.clear()
            break

        # Remove everything before 0xAA
        if start > 0:
            del buffer[:start]

        # At least 5 bytes are required to start parsing (AA + control + ID(2) + end).
        if len(buffer) < 5:
            break

        control = buffer[1]
        dlc = control & 0x0F  # data length (max 8)
        frame_len = 1 + 1 + 2 + dlc + 1  # AA + control + ID(2) + data + 55

        # Wait until the entire frame arrives
        if len(buffer) < frame_len:
            break

        frame = buffer[:frame_len]

        # Check that the last byte is 0x55 (end of frame)
        if frame[-1] == 0x55 and dlc <= 8:
            yield frame
            del buffer[:frame_len]
        else:
            del buffer[0]



def decode_frame(frame):
    control = frame[1]
    dlc = control & 0x0F
    can_id = frame[2] | (frame[3] << 8)
    data = frame[4:4 + dlc]
    return can_id, data


class CANBridgeNode(Node):
    def __init__(self):
        super().__init__('can_bridge')

        # Serial port
        self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.01, exclusive=False)
        self.buffer = bytearray()

        # ROS2 publishers/subscribers
        self.publisher = self.create_publisher(Frame, 'can_rx', 10)
        self.subscriber = self.create_subscription(Frame, 'can_tx', self.tx_callback, 10)

        self.timer = self.create_timer(0.01, self.read_serial)

        # Motors Initialisation
        self.init_motors()

    def send_message(self, can_id, data):
        """Sends a CAN frame via the serial port"""
        frame = build_framework(can_id, data)
        self.ser.write(frame)
        self.get_logger().info(f"[TX] {can_id:03X} [{' '.join(f'{b:02X}' for b in data)}]")

    def tx_callback(self, msg: Frame):
        """ROS2 callback when a node publishes on can_tx"""
        data = bytes(msg.data[:msg.dlc])  # keep only useful bytes
        self.send_message(msg.id, data)

    def init_motors(self):
        steps = [
            (0x000, [0x01, 0x01]),
            (0x000, [0x01, 0x02]),
            "pause",

            (0x601, [0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00]),
            (0x602, [0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00]),
            "pause",

            (0x601, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00]),
            (0x602, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00]),
            "pause",

            (0x601, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]),
            (0x602, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]),
            "pause",

            (0x601, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00]),
            (0x602, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00]),
            "pause",

            (0x601, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]),
            (0x602, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]),
            "pause"
        ]

        for step in steps:
            if step == "pause":
                time.sleep(0.5)
            else:
                can_id, data = step
                self.send_message(can_id, data)

        self.get_logger().info("Motors initialisation done")

    def read_serial(self):
        """Reads the serial port and publishes the received CAN frames on can_rx"""
        chunk = self.ser.read(64)
        if chunk:
            self.buffer.extend(chunk)
            for frame in read_work(self.buffer):
                can_id, data = decode_frame(frame)
                self.get_logger().info(f"[RX] {can_id:03X} [{' '.join(f'{b:02X}' for b in data)}]")

                msg = Frame()
                msg.id = can_id
                msg.is_extended = False
                msg.is_rtr = False
                msg.is_error = False
                msg.dlc = len(data)
                msg.data = list(data) + [0] * (8 - len(data))
                self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CANBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

