import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import sys
import time

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

class TestMovements(Node):

    """A simple controller that makes the robot stand up."""
    def __init__(self):
        super().__init__('movements')
        self.client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.joint_names = ['FL_HFE', 'FL_KFE', 'FR_HFE', 'FR_KFE', 'HL_HFE', 'HL_KFE', 'HR_HFE', 'HR_KFE']
        server_reached = self.client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to action server. Timeout exceeded.')
            sys.exit()

        self.joint_state = JointState()
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.js_sub

        # wait for the first joint state message
        if len(self.joint_state.position) == 0:
            self.get_logger().info('No Joint States received yet. Defaulting to 0.0 for all joints.')
            self.joint_state.position = [0.0] * 8

    def joint_state_callback(self, msg):
        self.joint_state = msg

    def sitdown(self):

        sitdown = {
            "name": "sitdown",
            "trajectory": [
                {
                    "positions": [
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [math.pi/4, -math.pi/2, math.pi/4, -math.pi/2, -math.pi/4, math.pi/2, -math.pi/4, math.pi/2],
                        [math.pi/2, -math.pi, math.pi/2, -math.pi, -math.pi/2, math.pi, -math.pi/2, math.pi],
                    ],
                    "durations": [2, 4, 6]
                }
            ]
        }

        self.execute(sitdown)

    def standup(self) -> None:
        standup = {
            "name": "standup",
            "trajectory": [
                {
                    "positions": [
                        [0, 0, 0, 0, 0, 0, 0, 0]
                    ],
                    "durations": [5]
                }
            ]
        }

        self.execute(standup)

    def test_legs(self) -> None:
        testlegs = {
            "name": "test_legs",
            "trajectory": [
                {
                    "positions": [
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [math.pi/2, -math.pi, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, math.pi/2, -math.pi, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, math.pi/2, -math.pi, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, math.pi/2, -math.pi],
                        [0, 0, 0, 0, 0, 0, 0, 0]
                    ],
                    "durations": [2, 8, 14, 20, 26, 32, 38, 44, 50]
                }
            ]
        }

        self.execute(testlegs)

    def jump(self) -> None:
        jump = {
            "name": "jump",
            "trajectory": [
                {
                    "positions": [
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [1, -2.2, 1, -2.2, -1, 2.2, -1, 2.2],
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [0.7, -1.5, 0.7, -1.5, -0.7, 1.5, -0.7, 1.5],
                        [0.7, -1.5, 0.7, -1.5, -0.7, 1.5, -0.7, 1.5],
                        [0, 0, 0, 0, 0, 0, 0, 0],
                    ],
                    "durations": [3, 4, 4.07, 4.4, 5.5, 6.5]
                }
            ]
        }

        self.execute(jump)

    def wave(self) -> None:
        dx = 0.38 # distance joints in x direction [m]
        u = 0.16 # upper leg and lower leg length [m]
        f = 2 # frequency [Hz]

        def wave_fct(t, phi):
            y_0 = 0.2
            y_hat = 0.1
            return y_0 + y_hat * math.sin(f*t + phi)

        def knee_angle(y):
            return math.asin(y/2/u)*2

        def hip_angle(y1, y2):
            front = math.acos(y1/2/u)
            back = math.acos(y2/2/u)

            return front, back

        def angles(t):
            yf = wave_fct(t, 0)
            yb = wave_fct(t, math.pi/2)

            k_f = knee_angle(yf) - math.pi
            k_b = knee_angle(yb) - math.pi

            h_f, h_b = hip_angle(yf, yb)

            return [h_f, k_f, h_f, k_f, h_b, k_b, h_b, k_b]

        steps = 20
        wave = {
            "name": "wave",
            "trajectory": [
                {
                    "positions": [[0]*8] + [angles(t) for t in range(steps)] + [[0]*8],
                    "durations": [i/2 for i in range(1, steps+3)]
                }
            ]
        }

        self.execute(wave)


    def execute(self, movement: dict) -> None:
        """
            execute movements using the joint trajectory controller

            a movement is a json style dictionary with the following structure:
            {
                "name": "movement_name",
                "trajectory": [
                    {
                        "positions": [[pos1], [pos2], ... ],
                        "durations": [durations]
                    }
                ]
            }

            each element of the positions array is a list of the joint positions. All 8 joints are required.
            the durations array contains the time from the start of the movement to the position in seconds.

            each movement starts from the current joint positions.
        """

        action = FollowJointTrajectory.Goal()
        action.trajectory.joint_names = self.joint_names
        action.trajectory.points = [JointTrajectoryPoint()]

        for trajectory in movement["trajectory"]:
            for i in range(len(trajectory["positions"])):
                point = JointTrajectoryPoint()
                point.positions = trajectory["positions"][i]
                point.time_from_start = rclpy.duration.Duration(seconds=trajectory["durations"][i]).to_msg()
                action.trajectory.points.append(point)

        # first positon is the current joint state
        # CAREFUL: They are not necessarily in the same order
        # we have to match the joint names
        action.trajectory.points[0].positions = [0.0] * 8
        for i in range(len(self.joint_state.name)):
            for j in range(len(self.joint_names)):
                if self.joint_state.name[i] == self.joint_names[j]:
                    action.trajectory.points[0].positions[j] = self.joint_state.position[i]

        action.trajectory.points[0].time_from_start = rclpy.duration.Duration(seconds=0).to_msg()

        self.get_logger().info(f'Executing movement {movement["name"]}...')
        future = self.client.send_goal_async(action)
        rclpy.spin_until_future_complete(self, future)

        actionfuture = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, actionfuture)
        self.get_logger().info(f'Movement {movement["name"]} executed successfully.')


def main(args=None):
    rclpy.init(args=args)

    move = TestMovements()

    for i in range(0, 1):
        move.sitdown()
        time.sleep(4)
        move.standup()
        time.sleep(4)

    move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()