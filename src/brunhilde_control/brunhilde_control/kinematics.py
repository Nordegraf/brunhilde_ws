import math


class Kinematics:
    def __init__(self):
        pass
    
    def inverse_kinematics(self,x, y, l1=160, l2=160):
            x = x*1000
            y = y*1000
        
            d = math.sqrt(x**2 + y**2)
            
            while d > (l1 + l2):
                print("Target is out of reach clipping edges")
                x = 0.99 * x
                y = 0.99 * y
                d = math.sqrt(x**2 + y**2)

            alpha = math.atan2(y, x)
            cos_theta1 = (d**2 - l1**2 - l2**2) / (2 * l1 * l2)
            sin_theta1 = math.sqrt(1 - cos_theta1**2) 
            theta1 = math.atan2(sin_theta1, cos_theta1)

            k1 = l1 + l2 * cos_theta1
            k2 = l2 * sin_theta1
            theta2 = alpha - math.atan2(k2, k1)
            

            return theta1, theta2
                

                
    def convert_trajectory(self, positions):
        joint_rad = []
        for position in positions:
            result = []
            for i in range(len(position)):
                theta1, theta2 = self.inverse_kinematics(position[i].x, position[i].z)
                result.append([theta1, theta2])
            
            joint_rad.append(result)

        final_joint_rad = []
        for i in range(len(positions[0])):
            final_joint_rad.append([
                joint_rad[0][i][0], joint_rad[0][i][1],
                joint_rad[1][i][0], joint_rad[1][i][1],
                joint_rad[2][i][0], joint_rad[2][i][1],
                joint_rad[3][i][0], joint_rad[3][i][1]
            ])

        return final_joint_rad