from __future__ import annotations

from geometry_msgs.msg import Twist
import numpy as np

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
    def get_movement_v2(self) -> tuple[TwistInterpretation.Act, TwistInterpretation.Turn]:
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