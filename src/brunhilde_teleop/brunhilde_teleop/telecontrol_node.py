import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy

import numpy as np
import math
import sys
import time

class TwistInterpretation:
    class Turn:
        NONE = 0
        LEFT = 1
        RIGHT = 2
    
    class Act:
        SIT = 0
        STAND = 1
        WALK = 2

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
    def get_movement_v2(self):
        act = 0
        if (self.walking):
            act = TwistInterpretation.Act.WALK
        else:
            act = TwistInterpretation.Act.STAND

        turn = 0
        if (self.turn_left):
            turn = TwistInterpretation.Turn.LEFT
        elif (self.turn_right):
            turn = TwistInterpretation.Turn.RIGHT
        else:
            turn = TwistInterpretation.Turn.NONE
        
        return (act, turn)

    
    @staticmethod
    def from_twist_msg(msg: Twist):
        lin = msg.linear
        ang = msg.angular
        updir = lin.z
        forward = lin.x
        turning = ang.z

        return TwistInterpretation(forward, turning, updir > 0,  updir < 0)

class TwistReader(Node):
    '''Notifies the robot controller of user activities (i.e. controlling movement)'''
    def __init__(self, twist_topic = '/cmd_vel') -> None:
        super().__init__('TeleControlTwistMonitor')
        self.subscription = self.create_subscription(Twist, twist_topic, self.callback, 10)

    def callback(msg: Twist):
        interp = TwistInterpretation.from_twist_msg(msg)

class RobotController:
    '''
    Transforms Twist messages into 
    TODO: might not be required!
    '''

class Action:
    def begin():
        # Obtain the array
        pass
    
    def try_cancel():
        # Get get_stop_thetas and execute that movement without possible cancellation
        # (Via different state in state engine)
        return None # next state
        pass


    @property
    def is_cancellable():
        pass

    def tick():
        # Cycle through the array
        # Send next request for movement to ros_control oder so 
        pass

class MovementServer(Node):
    '''
    Moves the robot according to the latest command. Should a command require intermediate movement, it shall also be done here.
    TODO: the command may be cancelled (although not every activity is cancelable), so this needs to cope with that
    '''

    
    def command_standup(self):
        '''Blocking action: lets the robot stand up.'''
        pass

    def command_sitdown(self):
        '''Blocking action: lets the robot sit down and prevents further movement inputs.'''
        pass

    def command_walk_(self):
        '''Asynchronous action: walks forward. Can be stopped at any time'''
        pass

    def callback(self):
        pass

class StateBroadcaster(Node):
    STR_TURN = {
        TwistInterpretation.Turn.LEFT: 'and turning left',
        TwistInterpretation.Turn.RIGHT: 'and turning right',
        TwistInterpretation.Turn.NONE: '',
    }
    STR_ACT = {
        TwistInterpretation.Act.SIT: 'sitting',
        TwistInterpretation.Act.STAND: 'standing',
        TwistInterpretation.Act.WALK: 'walking',
    }

    def __init__(self) -> None:
        super().__init__('StateBroadcaster')
        self.create_timer(1, self.timer_callback)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

        self.turn = TwistInterpretation.Turn.NONE
        self.act = TwistInterpretation.Act.SIT

    def timer_callback(self):
        print(self.STR_ACT[self.act], self.STR_TURN[self.turn])

    def twist_callback(self, msg: Twist):
        interp = TwistInterpretation.from_twist_msg(msg)
        (act, turn) = interp.get_movement_v2()
        self.act = act
        self.turn = turn
        print(msg)


def main(args=None):
    try:
        print('Brunhilde teleop status node')
        rclpy.init(args=args)

        twist_monitor = StateBroadcaster()
        rclpy.spin(twist_monitor)

        twist_monitor.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass