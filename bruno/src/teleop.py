#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys, select, tty, termios

# Key mappings: add/subtract delta to joint
KEYS = {
    'q': (0, 0.1), 'a': (0, -0.1),
    'w': (1, 0.1), 's': (1, -0.1),
    'e': (2, 0.1), 'd': (2, -0.1),
    'r': (3, 0.1), 'f': (3, -0.1),
    't': (4, 0.1), 'g': (4, -0.1),
    'y': (5, 0.1), 'h': (5, -0.1),
    'u': (6, 0.1), 'j': (6, -0.1),
    'i': (7, 0.1), 'k': (7, -0.1),
    'o': (8, 0.1), 'l': (8, -0.1),
    'p': (9, 0.1), ';': (9, -0.1),
    'z': (10, 0.1), 'x': (10, -0.1),
    'c': (11, 0.1), 'v': (11, -0.1),
}

class TeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.joint_positions = [0.0]*12
        self.settings = termios.tcgetattr(sys.stdin)
        print("Keyboard control started. Esc to quit.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == '\x1b':  # ESC
                    break
                elif key == ' ':
                    self.joint_positions = [0.0]*12
                elif key in KEYS:
                    idx, delta = KEYS[key]
                    self.joint_positions[idx] += delta

                msg = Float64MultiArray()
                msg.data = self.joint_positions
                self.pub.publish(msg)

                # Print joint positions
                sys.stdout.write("\r" + " | ".join(f"{j:.2f}" for j in self.joint_positions))
                sys.stdout.flush()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    node = TeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
