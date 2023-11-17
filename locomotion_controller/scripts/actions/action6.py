import numpy as np
from zmp_controller.TrajectoryGenerator import create_multiple_trajectory
import time
from zmp_controller.ikine import IKineQuadruped


class Action6():
    def __init__(self,
                 freq=300,
                 kinematic_scheme='x',
                 ef_init_x=0.149,
                 ef_init_y=0.13,
                 cog_offset_x=0.0,
                 cog_offset_y=0.0,
                 robot_height=0.17,
                 kp=12,
                 kd=0.1):
        
        self.freq = freq
        self.kinematic_scheme = kinematic_scheme
        self.ef_init_x = ef_init_x
        self.ef_init_y = ef_init_y
        self.robot_height = robot_height
        self.max_kp = kp
        self.max_kd = kd

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
        # self.cur_joint_pos = np.array([0.0]*12)
        # self.ref_joint_pos = np.array([0.0]*12)
        # self.ref_joint_vel = np.array([0.0]*12)
        # self.ref_joint_torq = np.array([0.0]*12)
        # self.ref_joint_kp = np.array([0.0]*12)
        # self.ref_joint_kd = np.array([0.0]*12)

        # for i in range(10):
        #     time.sleep(0.3)
        # Finish

        self.finished = True

    # Put your functions below