import numpy as np
from geometry_msgs.msg import Vector3



class BezierGait:
    STANCE = 0
    SWING = 1

    def __init__(self, dSref=[0.0,0.5,0.5, 0.0], dt=0.005, Tswing=0.2):
        self.dSref = dSref
        self.dt = dt
        self.Tswing = Tswing
        self.reset()
        self.NumControlPoints = 11

    def reset(self):
        self.time = 0.0
        self.TD_time = 0.0
        self.time_since_last_TD = 0.0
        self.StanceSwing = self.SWING
        self.SwRef = 0.0
        self.TD = False
        self.ref_idx = 0
        self.Phases = self.dSref
        self.Prev_fxyz = [np.zeros(3) for _ in range(4)]

    def get_phase(self, index, Tstance):
        Tstride = Tstance + self.Tswing
        ti = self.get_ti(index, Tstride)
        if ti < -self.Tswing:
            ti += Tstride

        if 0.0 <= ti <= Tstance:
            phase = ti / Tstance if Tstance > 0 else 0.0
            stance_swing = self.STANCE
        else:
            phase = (ti + self.Tswing) / self.Tswing if ti < 0 else (ti - Tstance) / self.Tswing
            stance_swing = self.SWING

        if index == self.ref_idx:
            self.StanceSwing = stance_swing
            if stance_swing == self.SWING:
                self.SwRef = phase
                if self.SwRef >= 0.999:
                    self.TD = True

        return phase, stance_swing

    def get_ti(self, index, Tstride):
        if index == self.ref_idx:
            self.dSref[index] = 0.0
        return self.time_since_last_TD - self.dSref[index] * Tstride

    def increment(self, dt, Tstride):
        self.check_touchdown()
        self.time_since_last_TD = self.time - self.TD_time
        if self.time_since_last_TD > Tstride:
            self.time_since_last_TD = Tstride
        elif self.time_since_last_TD < 0.0:
            self.time_since_last_TD = 0.0
        self.time += dt
        if Tstride < self.Tswing + dt:
            self.reset()

    def check_touchdown(self):
        if self.SwRef >= 0.9 and self.TD:
            self.TD_time = self.time
            self.TD = False
            self.SwRef = 0.0

    def bernstein_poly(self, t, k, point):
        return point * self.binomial(k) * (t ** k) * ((1 - t) ** (self.NumControlPoints - k))

    def binomial(self, k):
        return np.math.factorial(self.NumControlPoints) / (np.math.factorial(k) * np.math.factorial(self.NumControlPoints - k))

    def bezier_swing(self, phase, L, clearance_height, LateralFraction):
        X_POLAR = np.cos(LateralFraction)
        Y_POLAR = np.sin(LateralFraction)
        STEP = np.array([-L, -L*1.4, -L*1.5, -L*1.5, -L*1.5, 0.0, 0.0, 0.0, L*1.5, L*1.5, L*1.4, L])
        X = STEP * X_POLAR
        Y = STEP * Y_POLAR
        Z = np.array([0.0, 0.0, clearance_height * 0.9, clearance_height * 0.9, clearance_height * 0.9, clearance_height * 0.9, clearance_height * 0.9, clearance_height * 1.1, clearance_height * 1.1, clearance_height * 1.1, 0.0, 0.0])

        stepX, stepY, stepZ = 0.0, 0.0, 0.0
        for i in range(len(X)):
            stepX += self.bernstein_poly(phase, i, X[i])
            stepY += self.bernstein_poly(phase, i, Y[i])
            stepZ += self.bernstein_poly(phase, i, Z[i])

        return stepX, stepY, stepZ

    def sine_stance(self, phase, L, penetration_depth, LateralFraction):
        X_POLAR = np.cos(LateralFraction)
        Y_POLAR = np.sin(LateralFraction)
        step = L * (1.0 - 2.0 * phase)
        stepX = step * X_POLAR
        stepY = step * Y_POLAR
        stepZ = -penetration_depth * np.cos((np.pi * (stepX + stepY)) / (2.0 * L)) if L != 0.0 else 0.0
        return stepX, stepY, stepZ

    def yaw_circle(self, T_bf, index):
        DefaultBodyToFoot_Magnitude = np.linalg.norm([T_bf.x, T_bf.y])
        DefaultBodyToFoot_Direction = np.arctan2(T_bf.y, T_bf.x)
        g_xyz = self.Prev_fxyz[index] - np.array([T_bf.x, T_bf.y, T_bf.z])
        g_mag = np.linalg.norm(g_xyz[:2])
        th_mod = np.arctan2(g_mag, DefaultBodyToFoot_Magnitude)
        if index in [1, 2]:  # FR and BL
            phi_arc = np.pi / 2.0 + DefaultBodyToFoot_Direction + th_mod
        else:  # FL and BR
            phi_arc = np.pi / 2.0 - DefaultBodyToFoot_Direction + th_mod
        return phi_arc

    def swing_step(self, phase, L, T_bf,clearance_height, lateral):
        stepX, stepY, stepZ = self.bezier_swing(phase, L, clearance_height,lateral)
        stepX -= T_bf.x
        stepY -= T_bf.y
        stepZ -= T_bf.z
        return stepX, stepY, stepZ

    def stance_step(self, phase, L, T_bf,penetration_depth, lateral):
        stepX, stepY, stepZ = self.sine_stance(phase, L, penetration_depth , lateral)
        stepX -= T_bf.x
        stepY -= T_bf.y
        stepZ -= T_bf.z
        return stepX, stepY, stepZ

    def get_foot_step(self, phase, stance_swing, L, T_bf, clearance_height, penetration_depth, lateral):
        if stance_swing == self.SWING:
            return self.swing_step(phase, L, T_bf,clearance_height, lateral)
        else:
            return self.stance_step(phase, L, T_bf,penetration_depth, lateral)

    #for 12 dof LateralFractioon should be self.yaw_circle(T_bf, i)
    def generate_trajectory(self, T_bf, L, Tstance, dt, clearance_height, penetration_depth, lateral = 0):
        foot_positions = [[],[],[],[]]  
            
        
        Tstride = Tstance + self.Tswing
        
        while self.time <  Tstride:
            
            for i in range(len(foot_positions)):
                phase, stance_swing = self.get_phase(i, Tstance)
                stepX, stepY, stepZ = self.get_foot_step(phase, stance_swing, L, T_bf[i],clearance_height, penetration_depth, lateral)
                
                foot_step_vector = Vector3()
                foot_step_vector.x = -stepX
                foot_step_vector.y = stepY
                foot_step_vector.z = stepZ

                
                
                foot_positions[i].append(foot_step_vector)

            self.increment(dt, Tstride + self.Tswing)

        self.reset()  
        
        return foot_positions