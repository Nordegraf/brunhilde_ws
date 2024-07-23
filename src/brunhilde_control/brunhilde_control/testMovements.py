import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from rclpy.duration import Duration
from brunhilde_control.bezierGait import BezierGait
import sys
import numpy as np
import asyncio
import time
import math

class TestMovements(Node):

    def __init__(self):
        super().__init__('movements')
        self.client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.joint_names = ['FL_HFE', 'FL_KFE', 'FR_HFE', 'FR_KFE', 'HL_HFE', 'HL_KFE', 'HR_HFE', 'HR_KFE']
        self.default_body_to_foot = {}
        
        self.joint_state = JointState()
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        server_reached = self.client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to action server. Timeout exceeded.')
            sys.exit(1)

        # wait for the first joint state message
        if not self.joint_state.position:
            self.get_logger().info('No Joint States received yet. Defaulting to 0.0 for all joints.')
            self.joint_state.position = [0.0] * 8

    def update_transforms(self):
        self.default_body_to_foot['FR'] = self.get_transform('FR_FOOT', 'body_frame')
        self.default_body_to_foot['FL'] = self.get_transform('FL_FOOT', 'body_frame')
        self.default_body_to_foot['HR'] = self.get_transform('HR_FOOT', 'body_frame')
        self.default_body_to_foot['HL'] = self.get_transform('HL_FOOT', 'body_frame')

    def get_transform(self, target_frame, source_frame):
        try:
            tf_future = self.tf_buffer.wait_for_transform_async(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            rclpy.spin_until_future_complete(self, tf_future)          
            transform = asyncio.run(self.tf_buffer.lookup_transform_async(
                target_frame,
                source_frame,
                rclpy.time.Time()
            ))
            return transform
        except Exception as e:
            self.get_logger().info(f'Could not transform {target_frame} to {source_frame}: {e}')
            return None
        
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
        
    def walk(self,position) -> None:
        #could be changed but probably more sensible to change velocity in the trajectory generation
        durations = [2 * i for i in range(1, len(position) + 1)]
        
        walk = {
            "name": "walk",
            "trajectory": [
                {
                    "positions": position,
                    "durations": durations
                }
            ]
        }
        self.execute(walk)
    
    def joint_state_callback(self, msg):
        self.joint_state = msg

    def execute(self, movement: dict) -> None:
        action = FollowJointTrajectory.Goal()
        action.trajectory.joint_names = self.joint_names
        action.trajectory.points = [JointTrajectoryPoint()]

        for trajectory in movement["trajectory"]:
            for i in range(len(trajectory["positions"])):
                point = JointTrajectoryPoint()
                point.positions = trajectory["positions"][i]
                point.time_from_start = Duration(seconds=trajectory["durations"][i]).to_msg()
                action.trajectory.points.append(point)

        # first position is the current joint state
        # CAREFUL: They are not necessarily in the same order
        # we have to match the joint names
        action.trajectory.points[0].positions = [0.0] * 8
        for i in range(len(self.joint_state.name)):
            for j in range(len(self.joint_names)):
                if self.joint_state.name[i] == self.joint_names[j]:
                    action.trajectory.points[0].positions[j] = self.joint_state.position[i]

        action.trajectory.points[0].time_from_start = Duration(seconds=0).to_msg()

        self.get_logger().info(f'Executing movement {movement["name"]}...')
        future = self.client.send_goal_async(action)
        rclpy.spin_until_future_complete(self, future)

        action_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, action_future)
        self.get_logger().info(f'Movement {movement["name"]} executed successfully.')

    #is glaub 0.16 not sure though wie lange die beine sind im urdf und wie sich das genau überträgt
    def inverse_kinematics(self,x, y,l1=0.164,l2=0.16):      
       
        d = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
        
        if d > 1 or d < -1:
            print(f"Point ({x}, {y}) is out of reach")
            return 0.0,0.0
        
        theta2 = math.atan2(math.sqrt(1 - d**2), d)
        
        k1 = l1 + l2 * math.cos(theta2)
        k2 = l2 * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)
        
        return theta1, theta2
            
    def convert_trajectory(self, position):
        joint_rad = []
        for i in range(0,len(position)):
            theta1, theta2 = self.inverse_kinematics(position[i].x, position[i].y)    
            joint_rad.append(theta1)
            joint_rad.append(theta2)
        return joint_rad

def main(args=None):
    rclpy.init(args=args)

    move = TestMovements()
    move.update_transforms()
    T_bf = []
    for key, transform in move.default_body_to_foot.items():
        if transform:
            T_bf.append(transform.transform.translation)
        else:
            print(f"Transform for {key} is None")

    gait = BezierGait()
    
    for i in range(500):
        position = gait.generate_trajectory(T_bf,0.05,0.1,0.001)
        print(position)
        joint_rad = []
        joint_rad.append(move.convert_trajectory(position))
        T_bf = position
        print(joint_rad)
        move.walk(joint_rad)
        time.sleep(0.1)
    
    
    move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
