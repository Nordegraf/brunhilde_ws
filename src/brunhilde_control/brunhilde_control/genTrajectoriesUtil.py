from geometry_msgs.msg import Vector3
from brunhilde_control.bezierGait import BezierGait
from brunhilde_control.kinematics import Kinematics
import math


class GenTrajectoriesUtil():

    def __init__(self):
        self.gait = BezierGait()
        self.inv_kin = Kinematics()
        
        self.T_defaults = [
        Vector3(x=0.02134903039419904, y=-0.14795000000000008, z=0.2989693769141289),
        Vector3(x=0.02134903039419893, y=0.14795000000000003, z=0.2989693769141289),
        Vector3(x=0.02134903039419904, y=-0.14795000000000005, z=0.2989693769141289),
        Vector3(x=0.02134903039419904, y=0.14794999999999997, z=0.2989693769141289)
        ]
        self.startingPosition = [math.pi/4, -math.pi/2, math.pi/4, -math.pi/2, math.pi/4, -math.pi/2, math.pi/4, -math.pi/2]


    def walkGet(self):
        self.gait.dSref = [0.0, 0.5, 0.5, 0.0]
        position = self.gait.generate_trajectory(self.T_defaults,0.02,0.2,0.001, 0.001, 0.02)
        result = self.inv_kin.convert_trajectory(position)
        return result  
        
    def crawlGet(self):
        self.gait.dSref = [0.0, 0.5, 1.0, 1.5]
        position = self.gait.generate_trajectory(self.T_defaults,0.03,0.3,0.005, 0.002, 0.015)
        result = self.inv_kin.convert_trajectory(position)
        
    def trotGet(self):
        self.gait.dSref = [0.0, 0.0, 0.5, 0.5]
        position = self.gait.generate_trajectory(self.T_defaults,0.03,0.3,0.005, 0.002, 0.015)
        result = self.inv_kin.convert_trajectory(position)