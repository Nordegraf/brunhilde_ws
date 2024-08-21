from __future__ import annotations
from asyncio import Future
from typing import override
# from typing import NoReturn

from geometry_msgs.msg import Twist
import numpy as np

import rclpy.callback_groups
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
import math
import sys

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

BRUNHILDE_JOINT_NAMES = ['FL_HFE', 'FL_KFE', 'FR_HFE', 'FR_KFE', 'HL_HFE', 'HL_KFE', 'HR_HFE', 'HR_KFE']
class TimeoutArgs:
    '''special values for the timeout value of the function rclpy.spin_once'''
    DONT_WAIT = 0
    BLOCK_UNTIL_EVENT = None

# TODO: spin for duration actually sleeps, however, spin_once with a timeout could be called 
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

class PrefabTrajectory:
    def __init__(self, name, trajectory_positions: list[list[float]], trajectory_durations: list[float]) -> None:
        self.name = name
        self.trajectory_positions = trajectory_positions
        self.trajectory_durations = trajectory_durations

    def fulcrums(self):
        return zip(self.trajectory_positions, self.trajectory_durations)

class SimpleMovements:
    '''Here old movements based on FollowJointTrajectory.Goal are implemented'''
    class Prefabs:
        '''A collection of prefabricated trajectories'''
        Standup = PrefabTrajectory(
            "standup", 
            trajectory_positions=[[0, 0, 0, 0, 0, 0, 0, 0]], 
            trajectory_durations=[5]
        )

        Sitdown = PrefabTrajectory(
            "sitdown",
            trajectory_positions=[
                [0, 0, 0, 0, 0, 0, 0, 0],
                [math.pi/4, -math.pi/2, math.pi/4, -math.pi/2, -math.pi/4, math.pi/2, -math.pi/4, math.pi/2],
                [math.pi/2, -math.pi, math.pi/2, -math.pi, -math.pi/2, math.pi, -math.pi/2, math.pi],
            ], 
            trajectory_durations=[2, 4, 6]
        )

        SitdownFast = PrefabTrajectory(
            "fast sitdown",
            trajectory_positions=[
                [0, 0, 0, 0, 0, 0, 0, 0],
                [math.pi/4, -math.pi/2, math.pi/4, -math.pi/2, -math.pi/4, math.pi/2, -math.pi/4, math.pi/2],
                [math.pi/2, -math.pi, math.pi/2, -math.pi, -math.pi/2, math.pi, -math.pi/2, math.pi],
            ], 
            trajectory_durations=[1, 2, 3]
        )

    @staticmethod
    def execute(node: Node,  client: ActionClient, joint_names: list[str], current_joint_state: JointState, prefab: PrefabTrajectory):
        action = FollowJointTrajectory.Goal()
        action.trajectory.joint_names = joint_names
        action.trajectory.points = []

        # Firstly, set the start of the motion to be the current joint_state
        start_pos = JointTrajectoryPoint()
        start_pos.positions = [0.0] * len(joint_names)
        start_pos.time_from_start = Duration(seconds=0).to_msg()
        findindex = { x: i for i, x in zip(range(len(joint_names)), joint_names) }
        # The reference joint_names may have a different ordering than the ones in the given current joint state.
        # This uses the reference mapping
        for name, pos in zip(current_joint_state.name, current_joint_state.position):
            i = findindex[name]
            start_pos.positions[i] = pos
        action.trajectory.points.append(start_pos)

        # Then add the prefabricated points 
        # fulcrum = stützpunkt (auf einer Kurve/Trajektorie)
        # TODO: the name "duration" is wrong (its the delta from the start-time of the motion)
        for position, duration in prefab.fulcrums():
            # Ordering of BRUNHILDE_JOINT_NAMES is assumed
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = Duration(seconds=duration).to_msg()
            action.trajectory.points.append(point)

        node.get_logger().info(f'Executing movement {prefab.name}...')

        # Due to ROS being ROS, we get 2 futures:
        # - One that signals the goal has been sent
        # - One that signals the goal has been reached
        future = client.send_goal_async(action)
        # HOWEVER, due to ROS being ROS again, we reach a deadlock if we wait for said future here
        # THEREFORE, WE NEED TO RETURN IT ALL THE WAY TO THE STATE MACHINE LOOP
        future.add_done_callback(lambda _: node.get_logger().info(f'Movement {prefab.name} executed successfully.'))
        return future
        rclpy.spin_until_future_complete(node, future)

        actionfuture = future.result().get_result_async()
        rclpy.spin_until_future_complete(node, actionfuture)
        node.get_logger().info(f'Movement {prefab.name} executed successfully.')

    def execute2(node: Node, joint_state: BrunhildeJointState, prefab: PrefabTrajectory):
        # TODO: the obviously better way to do things around here
        pass

    @staticmethod
    def stand_up(node: Node,  client: ActionClient, joint_names: list[str], joint_state: JointState):
        return SimpleMovements.execute(node, client, joint_names, joint_state, SimpleMovements.Prefabs.Standup)

    @staticmethod
    def sit_down(node: Node,  client: ActionClient, joint_names: list[str], joint_state: JointState):
        return SimpleMovements.execute(node, client, joint_names, joint_state, SimpleMovements.Prefabs.Sitdown)

    @staticmethod
    def sit_down_fast(node: Node,  client: ActionClient, joint_names: list[str], joint_state: JointState):
        return SimpleMovements.execute(node, client, joint_names, joint_state, SimpleMovements.Prefabs.SitdownFast)


class State:
    def next_state(self, brunhilde: VirtualBrunhilde, delta: Duration,  msg: TwistInterpretation | None) -> State:
        raise NotImplemented("next_state was called, but is not implemented!")

    def next_critical_section(self, brunhilde: VirtualBrunhilde) -> None | Future:
        raise NotImplemented("next_critical_section was called, but is not implemented!")

class FollowTrajectoryState(State):
    def __init__(self, prefab: PrefabTrajectory) -> None:
        self.prefab = prefab
        self.future: Future | None = None
        self.finished = False
        self.started = False

    def begin(self, brunhilde: VirtualBrunhilde):
        fut: Future = brunhilde.execute_prefab(self.prefab)
        fut.add_done_callback(self.trajectory_begun_callback)
        self.started = True
        self.future = fut

    def trajectory_begun_callback(self, fut: Future):
        last_fut: Future =fut.result().get_result_async()
        last_fut.add_done_callback(self.trajectory_finished_callback)
        self.future = last_fut

    def trajectory_finished_callback(self, fut: Future):
        self.finished = True

    def next_state(self, brunhilde: VirtualBrunhilde, delta: Time, msg: TwistInterpretation | None):
        super().next_state(brunhilde, delta, msg)

    def next_critical_section(self, brunhilde: VirtualBrunhilde):
        if not self.started:
            self.begin(brunhilde)

class StandingUp(FollowTrajectoryState):
    def __init__(self) -> None:
        super().__init__(SimpleMovements.Prefabs.Standup)

    @property
    def stood_up(self):
        return self.finished

    def next_state(self, brunhilde: VirtualBrunhilde, delta: Time, msg: TwistInterpretation | None):
        match msg:
            case _ if self.stood_up: return IdleStanding()
            case None: return self
            case _: return self

class SittingDown(FollowTrajectoryState):
    def __init__(self) -> None:
        super().__init__(SimpleMovements.Prefabs.Sitdown)

    @property
    def sat_down(self):
        return self.finished

    def next_state(self, brunhilde: VirtualBrunhilde, delta: Time, msg: TwistInterpretation | None):
        match msg:
            case _ if self.sat_down: return IdleLow()
            case None: return self
            case _: return self

class IdleLow(State):
    def next_state(self, brunhilde: VirtualBrunhilde, delta: Time,  msg: TwistInterpretation | None):
        match msg:
            case None: return self
            case x if x.standup: return StandingUp()
            case _: return self

    def next_critical_section(self, brunhilde: VirtualBrunhilde):
        print('Nothing to be seen from below...')
        # spin_for_duration(brunhilde.node, Duration(seconds=1))
        return None

class StandingUpOld(State):
    def __init__(self) -> None:
        super().__init__()
        self.begun = False
        self.firstack = False
        self.stood_up = False
        self.future = None

    def next_state(self, brunhilde: VirtualBrunhilde, delta: Time, msg: TwistInterpretation | None):
        match msg:
            case _ if self.stood_up: return IdleStanding()
            case None: return self
            case _: return self

    def next_critical_section(self, brunhilde: VirtualBrunhilde):
        if not self.begun:
            print('begin standup')
            future = brunhilde.standup()
            future.add_done_callback(self.first_future_callback)
            self.begun = True
            self.future = future
    
    def first_future_callback(self, fut: Future):
        print('completed first future')
        self.firstack = True
        last_fut: Future = fut.result().get_result_async()
        last_fut.add_done_callback(self.second_future_callback)
        self.future = last_fut

    def second_future_callback(self, fut: Future):
        print('completed second future')
        self.stood_up = True
        self.future = None

class SittingDownOld(State):
    def __init__(self) -> None:
        super().__init__()
        self.begun = False
        self.firstack = False
        self.sat_down = False
        self.future = None

    def next_state(self, brunhilde: VirtualBrunhilde, delta: Time, msg: TwistInterpretation | None):
        match msg:
            case _ if self.sat_down: return IdleLow()
            case None: return self
            case _: return self

    def next_critical_section(self, brunhilde: VirtualBrunhilde):
        if not self.begun:
            print('begin sitdown')
            future = brunhilde.sitdown()
            future.add_done_callback(self.first_future_callback)
            self.begun = True
            self.future = future
    
    def first_future_callback(self, fut: Future):
        print('completed first future')
        self.firstack = True
        last_fut: Future = fut.result().get_result_async()
        last_fut.add_done_callback(self.second_future_callback)
        self.future = last_fut

    def second_future_callback(self, fut: Future):
        print('completed second future')
        self.sat_down = True
        self.future = None

class IdleStanding(State):
    def next_state(self, brunhilde: VirtualBrunhilde, delta: Time, msg: TwistInterpretation | None):
        if msg is None:
            return self

        signal = msg.get_state_signal()
        print('act-signal:', signal)

        match signal:
            case TwistInterpretation.Act.SITDOWN: return SittingDown()
            case TwistInterpretation.Act.WALK: raise NotImplementedError("Walking the botbot")
            case TwistInterpretation.Act.STANDUP | TwistInterpretation.Act.IDLE | _: return self
            # case x: raise NotImplementedError(f"Act signal [{x}] unkown")

    def next_critical_section(self, brunhilde: VirtualBrunhilde):
        print('Nothing to be seen from above...')
        # spin_for_duration(brunhilde.node, Duration(seconds=1))
        return None

class BeginWalk(State):
    def next_state(self, brunhilde: VirtualBrunhilde, delta: Time, msg: TwistInterpretation | None):
        pass

    def next_critical_section(self, brunhilde: VirtualBrunhilde):
        pass

class Walking(State):
    def next_state(self, brunhilde: VirtualBrunhilde, delta: Time, msg: TwistInterpretation | None):
        pass

    def next_critical_section(self, brunhilde: VirtualBrunhilde):
        pass

class StopWalk(State):
    def next_state(self, brunhilde: VirtualBrunhilde, delta: Time, msg: TwistInterpretation | None):
        pass

    def next_critical_section(self, brunhilde: VirtualBrunhilde):
        pass

class BrunhildeJointState:
    '''
    When dealing with joint states, the ordering/naming of the joints needs to be kept in mind.
    This class aims to alleviate the manual bookkeeping
    '''

    @staticmethod
    def from_description():
        pass

    @staticmethod
    def from_list(joint_positions, joint_names=BRUNHILDE_JOINT_NAMES):
        pass

    @staticmethod
    def build_trajectory(reference_names: list[str], positions: list[BrunhildeJointState], deltas: list[float]):
        '''
        A FollowJointTrajectory maps indexes to joint_names. 
        Therefore, every joint_position must follow the same index-scheme.
        This method builds a trajectory where the indexes of 'reference_names' correspond to the joint_name it indexes to for all the given entries in 'positions'.
        '''
        findindex = { x: i for i, x in zip(range(len(reference_names)), reference_names) }
        pass

class VirtualBrunhilde:
    #TODO: all the logic for executing movements (and waiting for their completion here)
    #TODO: let the execute function wrap the double-future in its own future (that waits for both)
    def __init__(self, node: Node, client: ActionClient, current_joint_state: JointState) -> None:
        self.node = node
        self.follow_joint_trajectory_client = client
        self.joint_state = current_joint_state

    def standup(self):
        return SimpleMovements.stand_up(
            node = self.node, 
            client = self.follow_joint_trajectory_client, 
            joint_names = BRUNHILDE_JOINT_NAMES, 
            joint_state = self.joint_state)

    def sitdown(self):
        return SimpleMovements.sit_down(
            node = self.node, 
            client = self.follow_joint_trajectory_client, 
            joint_names = BRUNHILDE_JOINT_NAMES, 
            joint_state = self.joint_state)

    def fast_sitdown(self):
        return SimpleMovements.sit_down_fast(
            node = self.node, 
            client = self.follow_joint_trajectory_client, 
            joint_names = BRUNHILDE_JOINT_NAMES, 
            joint_state = self.joint_state)

    def execute_prefab(self, prefab: PrefabTrajectory):
        return SimpleMovements.execute(
            node = self.node, 
            client = self.follow_joint_trajectory_client, 
            joint_names = BRUNHILDE_JOINT_NAMES, 
            current_joint_state = self.joint_state,
            prefab = prefab
        )

    def move_joints_to(self, position):
        pass

    def follow_trajectory(self, trajectory):
        pass


class WalkControllerNode(Node):
    '''A tele-operated controller that allows the robot to move, stand up, and sit down.'''
    def __init__(self, twist_topic = '/cmd_vel') -> None:
        super().__init__('WalkControllerNode')
        self.brunhilde = None
        self.future: Future | None = None

        # Callback Groups
        self.twist_cbg = None #MutuallyExclusiveCallbackGroup()
        self.client_cbg = None #MutuallyExclusiveCallbackGroup()
        self.timer_cbg = None #MutuallyExclusiveCallbackGroup()

        ### Teleoperation ###
        self.twist_subscription = self.create_subscription(Twist, twist_topic, self.twist_callback, 1, callback_group=self.twist_cbg)
        self.twist_timeout_seconds = 0.5
        self.last_msg_interpreted: (TwistInterpretation | None) = None
        self.remaining_msg_time: float = 0

        ### Robot control ###
        # Joint control
        self.client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory', callback_group=self.client_cbg)
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
            self.joint_state.name = BRUNHILDE_JOINT_NAMES
            self.joint_state.position = [0.0] * 8

        ### State machine ###
        self.state: State = IdleLow()
        self.last_critical_start: Time = self.get_clock().now()

        # The state machine needs to be called into periodically instead of being trampolined (i.e.: while True: next_step()).
        # This is due to the fact, that spinning and waiting for futures in the trampoline yields to the following deadlock:
        # No event can be processed (for unknown reasons), but we need to spin to obtain JointState messages (or stuff from the client -> unsure).
        # Without that, everything comes to a halt.
        # Furthermore: spinning inside of a timer breaks the timer; it will not be called again.
        # Therefore: All the async stuff needs to be done via manual polling after all, since there is no Ros2 behaviour that can be depended on.
        self.timer_interval = 0.050
        self.timer_autostart = False
        self.state_machine_timer = self.create_timer(self.timer_interval, self.timer_callback, callback_group=self.timer_cbg, autostart=self.timer_autostart)

        # rclpy doesnt have spin_sone, so we need to wing it by having the state_machine be a timed event instead of it asking whenever it's ready
        self.in_state_step = False

        ### Virtual Brunhilde ###
        self.brunhilde = VirtualBrunhilde(self, self.client, self.joint_state)

    def check_twist(self):
        '''If an event occured, it will be processed here. If not, this method returns immediately.'''
        # rclpy doesn't have spin_some, which spins until the message queue is idle
        # So we have to pretend we can do that here: In essense we just check as often as the queue is large:
        rclpy.spin_once(self, timeout_sec=TimeoutArgs.DONT_WAIT)
        # Actually, the state-machine step is now just a timed event, so FU ros

    def twist_callback(self, msg: Twist):
        '''
        Interprets the twist message, such that the state machine can work with it in an easier fashion.

        sets self.last_msg_interpreted
        '''
        interp = TwistInterpretation.from_twist_msg(msg)
        self.last_msg_interpreted = interp
        self.remaining_msg_time = self.twist_timeout_seconds
        print('TWIST_CALLBACK', msg)

    def joint_state_callback(self, msg: JointState):
        '''
        The joint state of brunhilde is updated through this callback.

        sets brunhilde.joint_state
        '''
        self.joint_state = msg
        match self.brunhilde:
            case None: pass
            case b: b.joint_state = msg
        # print('JOINT_STATE_CALLBACK')

    def timer_callback(self):
        now = self.get_clock().now()
        delta: Duration = now - self.last_critical_start
        delta_seconds =  delta.nanoseconds / 1e9
        next_state = self.state.next_state(self.brunhilde, delta, self.last_msg_interpreted)
        self.remaining_msg_time = self.remaining_msg_time - delta_seconds
        
        if self.state is not next_state:
            print('old state:', self.state)
            print('new state:', next_state)
            print('last msg:', self.last_msg_interpreted)
            self.last_msg_interpreted = None
        elif self.last_msg_interpreted is not None and self.remaining_msg_time < 0.0:
            print('last msg:', self.last_msg_interpreted)
            print('TIMED OUT TWIST MESSAGE')

        self.state  = next_state
        self.state.next_critical_section(self.brunhilde)

    # @NoReturn
    def run_state_machine(self):
        '''
        Starts running the state machine.
        Repeats the following steps in order:

        - get the next state from state.next_state
        - enter state.next_critical_section
        - check for at least one ros-message via rclpy.spin_once
        '''

        self.state_machine_timer.reset()
        rclpy.spin(self)

        # self.state_machine_timer.reset()
        # executor = MultiThreadedExecutor()
        # executor.add_node(self)
        # executor.spin()
        print("I shouldn't be here")
        # rclpy.spin(self)
        return

        while True:
            now = self.get_clock().now()
            delta: Duration = now - self.last_critical_start
            next_state = self.state.next_state(self.brunhilde, delta, self.last_msg_interpreted)


            print('old state:', self.state)
            print('new state:', next_state)
            print('last msg:', self.last_msg_interpreted)
            
            if self.state is not next_state:
                print('...cleaned last msg')
                self.last_msg_interpreted = None


            self.state  = next_state
            self.state.next_critical_section(self.brunhilde)
            self.check_twist()


def main(args=None):
    try:
        print('Brunhilde teleop walk node')
        rclpy.init(args=args)
        
        node = WalkControllerNode()
        # rclpy.spin(node)
        # TODO: perhaps get to know the start state already?
        #       ^ better: wait for config status message
        # node.brunhilde.sitdown()
        # node.brunhilde.standup()
        # node.brunhilde.sitdown()
        node.state = IdleLow()
        node.run_state_machine()

        # node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        # rclpy.shutdown()