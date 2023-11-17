import numpy as np
from zmp_controller.TrajectoryGenerator import create_multiple_trajectory
import time
from zmp_controller.ikine import IKineQuadruped


class LayDown():
    def __init__(self,
                 freq=300,
                 kinematic_scheme='x',
                 ef_init_x=0.149,
                 ef_init_y=0.13,
                 cog_offset_x=0.0,
                 cog_offset_y=0.0,
                 robot_height=0.17,
                 kp=[12, 12, 12],
                 kd=[0.1, 0.1, 0.1]):
        
        self.freq = freq
        self.kinematic_scheme = kinematic_scheme
        self.ef_init_x = ef_init_x
        self.ef_init_y = ef_init_y
        self.cog_offset_x = cog_offset_x
        self.cog_offset_y = cog_offset_y
        self.robot_height = robot_height
        self.max_kp = np.array(kp[:3])
        self.max_kd = np.array(kd[:3])
        self.kp = kp
        self.kd = kd

        self.finished = False
        self.cur_joint_pos = np.array([0.0]*12)
        self.ref_joint_pos = np.array([0.0]*12)
        self.ref_joint_vel = np.array([0.0]*12)
        self.ref_joint_torq = np.array([0.0]*12)
        self.ref_joint_kp = np.array([0.0]*12)
        self.ref_joint_kd = np.array([0.0]*12)

        self.ik = IKineQuadruped(theta_offset=[0, -np.pi/2, 0])

    def is_finished(self):
        return self.finished

    def set_cur_joint_pos(self, cur_pos):
        self.cur_joint_pos = cur_pos[:]

    def get_ref_joint_pos(self):
        return self.ref_joint_pos, self.ref_joint_vel, self.ref_joint_torq, self.ref_joint_kp, self.ref_joint_kd

    def execute(self):
        self.finished = False

        # Put your code here
        self.ref_joint_kp = np.array(self.kp, dtype=float)
        self.ref_joint_kd = np.array(self.kd, dtype=float)

        # theta_cur = np.array(self.cur_joint_pos[:])
        # theta_ref = self.ik.calculate([ self.ef_init_x, -self.ef_init_y, -self.robot_height,
        #                                 self.ef_init_x,  self.ef_init_y, -self.robot_height,
        #                                 -self.ef_init_x, -self.ef_init_y, -self.robot_height,
        #                                 -self.ef_init_x,  self.ef_init_y, -self.robot_height], config=self.kinematic_scheme)
        # theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 1.0, 1/self.freq)
        # self.take_position(theta_refs)

        # time.sleep(1)

        theta_cur = np.array(self.cur_joint_pos[:])
        # self.theta_ref = self.theta_cur[:]
        if self.kinematic_scheme =='m':
            theta_ref = [theta_cur[0], -1.57, 2.9, theta_cur[3], 1.57, -2.9, theta_cur[6], -1.57, 2.9, theta_cur[9], 1.57, -2.9]
        elif self.kinematic_scheme == 'x':
            theta_ref = [theta_cur[0], -1.57, 2.9, theta_cur[3], 1.57, -2.9, theta_cur[6], 1.57, -2.9, theta_cur[9], -1.57, 2.9]
        elif self.kinematic_scheme =='o':
            theta_ref = [theta_cur[0], 1.57, -2.9, theta_cur[3], -1.57, 2.9, theta_cur[6], -1.57, 2.9, theta_cur[9], 1.57, -2.9]

        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.5, 1/self.freq)
        self.take_position(theta_refs)

        theta_cur = np.array(self.cur_joint_pos[:])
        # self.theta_ref = self.theta_cur[:]
        if self.kinematic_scheme =='m':
            self.theta_ref = [0, -1.57, 2.9, 0, 1.57, -2.9, 0, -1.57, 2.9, 0, 1.57, -2.9]
        elif self.kinematic_scheme == 'x':
            self.theta_ref = [0, -1.57, 2.9, 0, 1.57, -2.9, 0, 1.57, -2.9, 0, -1.57, 2.9]
        elif self.kinematic_scheme =='o':
            self.theta_ref = [0, 1.57, -2.9, 0, -1.57, 2.9, 0, -1.57, 2.9, 0, 1.57, -2.9]
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.5, 1/self.freq)
        self.take_position(theta_refs)

        # Decrease kpkd
        for _ in range(int(self.freq*0.2)):
            self.kpkd_dec(0.2)
            # print(self.ref_joint_kp)
        self.ref_joint_kp = np.array([0]*12)
        self.ref_joint_kd = np.array([0]*12)

        # Finish

        self.finished = True

    # Put your functions below
    def take_position(self, theta_refs):
        for i in range(len(theta_refs[0])):
            for j in range(12):
                self.ref_joint_pos[j] = theta_refs[j][i]

            time.sleep(1/self.freq)

    def kpkd_dec(self, t):

        ts = self.freq*t
        kp_dec = self.max_kp/ts
        kd_dec = self.max_kd/ts
        # print(kp_dec, ts, self.freq, t, self.max_kp)

        for i in range(3):
            if self.ref_joint_kp[i] > 0.0:
                self.ref_joint_kp[i]   -= kp_dec[i]
                self.ref_joint_kp[i+3] -= kp_dec[i]
                self.ref_joint_kp[i+6] -= kp_dec[i]
                self.ref_joint_kp[i+9] -= kp_dec[i]
            if self.ref_joint_kp[i] < 0.0:
                self.ref_joint_kp[i]   = 0
                self.ref_joint_kp[i+3] = 0
                self.ref_joint_kp[i+6] = 0
                self.ref_joint_kp[i+9] = 0

            if self.ref_joint_kd[i] > 0.0:
                self.ref_joint_kd[i]   -= kd_dec[i]
                self.ref_joint_kd[i+3] -= kd_dec[i]
                self.ref_joint_kd[i+6] -= kd_dec[i]
                self.ref_joint_kd[i+9] -= kd_dec[i]
            if self.ref_joint_kd[i] < 0.0:
                self.ref_joint_kd[i]   = 0
                self.ref_joint_kd[i+3] = 0
                self.ref_joint_kd[i+6] = 0
                self.ref_joint_kd[i+9] = 0

        time.sleep(1/self.freq)