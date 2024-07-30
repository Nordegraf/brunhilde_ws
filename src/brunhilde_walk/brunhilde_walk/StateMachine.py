from __future__ import annotations

from brunhilde_walk.brunhilde_walk.TwistInterpretation import *

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.duration import Duration

def spin_for_duration(node: Node, d: Duration):
    node.get_clock().sleep_for()

class State:
    def next_state(self, node: Node, delta: Time,  msg: TwistInterpretation | None) -> State:
        raise NotImplemented("next_state was called, but is not implemented!")

    def next_critical_section(self, node: Node):
        raise NotImplemented("next_critical_section was called, but is not implemented!")
    
    # def ignore_input(self, msg):
    #     return self, None
    
    # def no_operation(self, msg):
    #     return self, msg

class IdleLow(State):
    def next_state(self, node: Node, delta: Time,  msg: TwistInterpretation | None):
        if msg.standup:
            return StandingUp()
        else:
            return self
    
    def next_critical_section(self, node: Node):
        print('Nothing to be seen from below...')
        spin_for_duration(node, Duration(seconds=1))

class StandingUp(State):
    def __init__(self) -> None:
        super().__init__()
        self.stood_up = False

    def next_state(self, node: Node, delta: Time, msg: TwistInterpretation | None):
        if self.stood_up:
            return IdleStanding()
        else:
            return self
    
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
            return IdleStanding()
        else:
            return self
    
    def next_critical_section(self, node: Node):
        self.sat_down = True
        print('Sitting down')
        spin_for_duration(node, Duration(seconds=1))
    
class IdleStanding(State):
    CONTINUATIONS = {
        TwistInterpretation.Act.SIT: lambda: SittingDown(),
        # TwistInterpretation.Act.WALK: lambda: BeginWalk(),
    }

    def next_state(self, node: Node, delta: Time, msg: TwistInterpretation | None):
        act, turn = msg.get_movement_v2()

        if act in self.CONTINUATIONS:
            return self.CONTINUATIONS[act]()
        else:
            return self
    
    def next_critical_section(self, node: Node):
        print('Nothing to be seen from above...')
        spin_for_duration(node, Duration(seconds=0.5))