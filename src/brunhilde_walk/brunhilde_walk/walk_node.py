import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy

import numpy as np

from brunhilde_walk.brunhilde_walk.TwistInterpretation import TwistInterpretation
from brunhilde_walk.brunhilde_walk.StateMachine import State
import brunhilde_walk.brunhilde_walk.StateMachine as States

DONT_WAIT = 0

class WalkNode(Node):

    def __init__(self, twist_topic = '/cmd_vel') -> None:
        super().__init__('WalkNode')
        self.twist_subscription = self.create_subscription(Twist, twist_topic, self.twist_callback, 1)

        self.state: State = States.IdleLow()
        self.last_msg_interpreted: (TwistInterpretation | None) = None
        self.last_critical_start: Time = self.get_clock().now()

    def check_twist(self):
        rclpy.spin_once(DONT_WAIT)


    def timer_callback(self):
        print(self.STR_ACT[self.act], self.STR_TURN[self.turn])

    def twist_callback(self, msg: Twist):
        interp = TwistInterpretation.from_twist_msg(msg)
        self.last_msg_interpreted = interp
        # TODO: start timer: after x ms => set interp to none, cancelled by re-entry of twist_callback
        
        # (act, turn) = interp.get_movement_v2()
        # self.act = act
        # self.turn = turn
        print(msg)

    def run_state_machine(self):
        while True:
            now = self.get_clock().now()
            delta: Duration = now - self.last_critical_start
            self.state, self.last_msg_interpreted = self.state.next_state(self, delta, self.last_msg_interpreted)
            self.state.next_critical_section(self)
            self.check_twist()

def main(args=None):
    try:
        print('Brunhilde teleop walk node')
        rclpy.init(args=args)
        
        node = WalkNode()
        # rclpy.spin(node)
        node.run_state_machine()

        # node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()