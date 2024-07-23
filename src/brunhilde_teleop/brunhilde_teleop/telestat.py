import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import numpy as np




class TwistInterpretation:
    def __init__(self, walk: float, turn: float, standup: bool, sitdown: bool) -> None:
        self.walk = float(walk)
        self.turn = float(turn)
        self.standup = standup
        self.sitdown = sitdown

    def turn_rate(self):
        return np.abs(self.turn)

    def turn_left(self):
        return self.turn > 0
    
    def turn_right(self):
        return self.turn < 0
    
    def turning(self):
        return self.turn != 0

    def go_forward(self):
        return self.walk > 0
    
    def go_backward(self):
        return self.walk < 0

    def walking_speed(self):
        return np.abs(self.walk)

    def walking(self):
        return self.walk != 0
    
    def from_twist_msg(msg: Twist):
        pass


def decode_msg(msg: Twist):
    lin = msg.linear
    ang = msg.angular
    updir = lin.z
    forward = lin.x
    turning = ang.z

    return TwistInterpretation(forward, turning, updir > 0,  updir < 0)


def print_action(act: TwistInterpretation):
    # TODO: backwards left/right is inverted in keyboard-twist
    if act.walk != 0:
        print('Movement: Walking ', 'forward' if act.go_forward() else 'backward')
    else:
        print('Movement: Standing still')
    if act.turning():
        print('while turning: ', 'left' if act.turn_left() else 'right')
    else:
        print('while keeping orientation')
    print()
    if act.standup:
        print('Standup request')
    elif act.sitdown:
        print('Sitdown request')
    else:
        pass

class TwistMonitor(Node):
    def __init__(self) -> None:
        super().__init__('TwistMonitor')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)


    def callback(self, msg: Twist):
        act = decode_msg(msg)
        print_action(act)

def main(args=None):
    print('Brunhilde teleop status node')
    rclpy.init(args=args)

    twist_monitor = TwistMonitor()
    rclpy.spin(twist_monitor)

    twist_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
