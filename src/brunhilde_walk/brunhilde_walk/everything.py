from __future__ import annotations
from typing import NoReturn

from geometry_msgs.msg import Twist
import numpy as np

from rclpy.time import Time
from rclpy.node import Node
from rclpy.duration import Duration

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
import sys

def spin_for_duration(node: Node, d: Duration):
    node.get_clock().sleep_for(d)

class TwistInterpretation:
    class Turn:
        NONE = None
        LEFT = None
        RIGHT = None

        def __init__(self, val) -> None:
            self.val = val

        def __repr__(self) -> str:
            return ['NONE', 'LEFT', 'RIGHT'][self.val]
    
    Turn.NONE = Turn(0)
    Turn.LEFT = Turn(1)
    Turn.RIGHT = Turn(2)

    class Act:
        IDLE = None
        SITDOWN = None
        STANDUP = None
        WALK = None

        def __init__(self, val) -> None:
            self.val = val

        def __repr__(self) -> str:
            return ['IDLE', 'SIT', 'STAND', 'WALK'][self.val]

    Act.IDLE = Act(0)
    Act.SITDOWN = Act(1)
    Act.STANDUP = Act(2)
    Act.WALK = Act(3)

    def __init__(self, walk: float, turn: float, standup: bool, sitdown: bool) -> None:
        self.walk = float(walk)
        self.turn = float(turn)
        self.standup = standup
        self.sitdown = sitdown
    
    @property
    def turn_rate(self):
        return np.abs(self.turn)

    @property
    def turn_left(self):
        return self.turn > 0
    
    @property
    def turn_right(self):
        return self.turn < 0
        
    @property
    def turning(self):
        return self.turn != 0
    
    @property
    def go_forward(self):
        return self.walk > 0
        
    @property
    def go_backward(self):
        return self.walk < 0
    
    @property
    def walking_speed(self):
        return np.abs(self.walk)
    
    @property
    def walking(self):
        return self.walk != 0
        
    # @property
    def get_movement_v2(self) -> tuple[TwistInterpretation.Act, TwistInterpretation.Turn]:
        # TODO: proper states WALK/STAND is should be mutually exclusive to SIT
        act = 0
        if (self.walking):
            act = TwistInterpretation.Act.WALK
        else:
            act = TwistInterpretation.Act.STANDUP

        turn = 0
        if (self.turn_left):
            turn = TwistInterpretation.Turn.LEFT
        elif (self.turn_right):
            turn = TwistInterpretation.Turn.RIGHT
        else:
            turn = TwistInterpretation.Turn.NONE
        
        return (act, turn)

    def get_state_signal(self):
        if self.standup:
            return TwistInterpretation.Act.STANDUP
        elif self.sitdown:
            return TwistInterpretation.Act.SITDOWN
        elif self.walking:
            return TwistInterpretation.Act.WALK
        else:
            return TwistInterpretation.Act.IDLE

    def __repr__(self) -> str:
        (act, turn) = self.get_movement_v2()
        return f"Twist(act= {act}, turn={turn}, standup: {self.standup}, sitdown: {self.sitdown})"
    
    @staticmethod
    def from_twist_msg(msg: Twist):
        lin = msg.linear
        ang = msg.angular
        updir = lin.z
        forward = lin.x
        turning = ang.z

        return TwistInterpretation(forward, turning, updir > 0,  updir < 0)
    

class State:
    def next_state(self, node: Node, delta: Time,  msg: TwistInterpretation | None) -> State:
        raise NotImplemented("next_state was called, but is not implemented!")

    def next_critical_section(self, node: Node):
        raise NotImplemented("next_critical_section was called, but is not implemented!")

class IdleLow(State):
    def next_state(self, node: Node, delta: Time,  msg: TwistInterpretation | None):
        match msg:
            case None: return self
            case x if x.standup: return StandingUp()
            case _: return self
    
    def next_critical_section(self, node: Node):
        print('Nothing to be seen from below...')
        spin_for_duration(node, Duration(seconds=1))
        # TODO: spin for duration actually sleeps, however, spin_once with a timeout could be called 

class StandingUp(State):
    def __init__(self) -> None:
        super().__init__()
        self.stood_up = False

    def next_state(self, node: Node, delta: Time, msg: TwistInterpretation | None):
        match msg:
            case _ if self.stood_up: return IdleStanding()
            case None: return self
            case _: return self
    
    def next_critical_section(self, node: Node):
        self.stood_up = True
        print('Standing up')
        spin_for_duration(node, Duration(seconds=1))
    
class SittingDown(State):
    def __init__(self) -> None:
        super().__init__()
        self.sat_down = False

    def next_state(self, node: Node, delta: Time, msg: TwistInterpretation | None):
        if self.sat_down:
            return IdleLow()
        elif msg is None:
            return self
        else:
            return self
    
    def next_critical_section(self, node: Node):
        self.sat_down = True
        print('Sitting down')
        spin_for_duration(node, Duration(seconds=1))
    
class IdleStanding(State):
    def next_state(self, node: Node, delta: Time, msg: TwistInterpretation | None):
        if msg is None:
            return self

        signal = msg.get_state_signal()
        print('ACT:', signal)
        print('CHOICE:', self.CONTINUATIONS)

        match signal:
            case TwistInterpretation.Act.SITDOWN: return SittingDown()
            case TwistInterpretation.Act.WALK: raise NotImplementedError("Walking the botbot")
            case TwistInterpretation.Act.STANDUP | TwistInterpretation.Act.IDLE | _: return self

    
    def next_critical_section(self, node: Node):
        print('Nothing to be seen from above...')
        spin_for_duration(node, Duration(seconds=1))

class VirtualBrunhilde:
    #TODO: all the logic for executing movements (and waiting for their completion here)
    #TODO: let the execute function wrap the double-future in its own future (that waits for both)
    
    def standup(self):
        pass
        #TODO: return a future that rclpy can spinwait on

    def sitdown(self):
        pass

    def move_to(self, position):
        pass

    def follow_trajectory(self, trajectory):
        pass

BRUNHILDE_JOINT_NAMES = ['FL_HFE', 'FL_KFE', 'FR_HFE', 'FR_KFE', 'HL_HFE', 'HL_KFE', 'HR_HFE', 'HR_KFE']
DONT_WAIT = 0

class WalkControllerNode(Node):
    '''A tele-operated controller that allows the robot to move, stand up, and sit down.'''
    def __init__(self, twist_topic = '/cmd_vel') -> None:
        super().__init__('WalkControllerNode')

        ### Teleoperation ###
        self.twist_subscription = self.create_subscription(Twist, twist_topic, self.twist_callback, 1)

        ### Robot control ###
        # Joint control
        self.client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.joint_names = BRUNHILDE_JOINT_NAMES
        server_reached = self.client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to action server. Timeout exceeded.')
            sys.exit()
        # Joint states
        self.joint_state = JointState()
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # wait for the first joint state message
        if len(self.joint_state.position) == 0:
            self.get_logger().info('No Joint States received yet. Defaulting to 0.0 for all joints.')
            self.joint_state.position = [0.0] * 8

        ### State machine ###
        self.state: State = IdleLow()
        self.last_msg_interpreted: (TwistInterpretation | None) = None
        self.last_critical_start: Time = self.get_clock().now()

    def check_twist(self):
        rclpy.spin_once(self, timeout_sec=DONT_WAIT)

    def twist_callback(self, msg: Twist):
        interp = TwistInterpretation.from_twist_msg(msg)
        self.last_msg_interpreted = interp
        # TODO: start timer: after x ms => set interp to none, cancelled by re-entry of twist_callback
        
        # (act, turn) = interp.get_movement_v2()
        # self.act = act
        # self.turn = turn
        print(msg)

    def joint_state_callback(self, msg):
        self.joint_state = msg

    @NoReturn
    def run_state_machine(self):
        while True:
            now = self.get_clock().now()
            delta: Duration = now - self.last_critical_start
            next_state = self.state.next_state(self, delta, self.last_msg_interpreted)


            print('old state:', self.state)
            print('new state:', next_state)
            print('last msg:', self.last_msg_interpreted)
            
            if self.state is not next_state:
                print('...cleaned last msg')
                self.last_msg_interpreted = None


            self.state  = next_state
            self.state.next_critical_section(self)
            self.check_twist()


def main(args=None):
    try:
        print('Brunhilde teleop walk node')
        rclpy.init(args=args)
        
        node = WalkControllerNode()
        # rclpy.spin(node)
        node.run_state_machine()

        # node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()