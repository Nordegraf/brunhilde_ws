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
        pass
    
    def try_cancel():
        pass


    @property
    def is_cancellable():
        pass

    def tick():
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

def main(args=None):
    pass

if __name__ == '__main__':
    main()