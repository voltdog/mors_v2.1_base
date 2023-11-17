import numpy as np
from zmp_controller.TrajectoryGenerator import create_multiple_trajectory
import time
from zmp_controller.ikine import IKineQuadruped
from zmp_controller.fkine import FKineQuadruped


class Action5():
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
        self.fk = FKineQuadruped(theta_offset=[0, -np.pi/2, 0])

    def is_finished(self):
        return self.finished

    def set_cur_joint_pos(self, cur_pos):
        self.cur_joint_pos = cur_pos[:]

    def get_ref_joint_pos(self):
        return self.ref_joint_pos, self.ref_joint_vel, self.ref_joint_torq, self.ref_joint_kp, self.ref_joint_kd

    def take_position(self, theta_refs):
        for i in range(len(theta_refs[0])):
            for j in range(12):
                self.ref_joint_pos[j] = theta_refs[j][i]

            time.sleep(1/self.freq)

    def execute(self):
        self.finished = False

        # Put your code here
        
        self.ref_joint_kp = np.array(self.kp, dtype=float)
        self.ref_joint_kd = np.array(self.kd, dtype=float)


        self.ref_joint_pos = np.array(self.cur_joint_pos[:])
        time.sleep(0.1)

        # sit down
        theta_cur = np.array(self.cur_joint_pos[:])
        theta_ref = [0.3, -0.57, 0.31, 
                      -0.3, 0.57, -0.31, 
                      -0.34, -1.29, 2.28, 
                      0.34, 1.29, -2.28]
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.5, 1/self.freq)
        self.take_position(theta_refs)

        # time.sleep(2)

        # raise hand
        # time.sleep(2)

        theta_cur = theta_ref[:]

        l2_cur_pos = self.fk.calculate(self.cur_joint_pos)[3:6]
        theta_ref[3:6] = self.ik.ikine_L1([l2_cur_pos[0], l2_cur_pos[1]+0.02, l2_cur_pos[2]+0.04], config=self.kinematic_scheme)
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.2, 1/self.freq)
        self.take_position(theta_refs)

        # time.sleep(2)

        theta_cur = theta_ref[:]
        theta_ref = [0.3, -0.57, 0.31, 
                      -0.1, -2.89, 0.52, 
                      -0.34, -1.29, 2.28, 
                      0.34, 1.29, -2.28]
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.6, 1/self.freq)
        self.take_position(theta_refs)

        
        # time.sleep(2)

        for i in range(3):
            # move L2 left
            theta_cur = theta_ref[:]
            theta_ref = [0.3, -0.57, 0.31, 
                          0.3, -2.89, 0.52, 
                          -0.34, -1.29, 2.28, 
                          0.34, 1.29, -2.28]
            theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.15, 1/self.freq)
            self.take_position(theta_refs)

            # move L2 right
            theta_cur = theta_ref[:]
            theta_ref = [0.3, -0.57, 0.31, 
                          -0.1, -2.89, 0.52, 
                          -0.34, -1.29, 2.28, 
                          0.34, 1.29, -2.28]
            theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.15, 1/self.freq)
            self.take_position(theta_refs)

        theta_cur = theta_ref[:]
        theta_ref = [0.3, -0.57, 0.31, 
                      -0.3, 0.57, -0.31, 
                      -0.34, -1.29, 2.28, 
                      0.34, 1.29, -2.28]
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.6, 1/self.freq)
        self.take_position(theta_refs)

        # get L2 foot position
        # l2_cur_pos = self.fk.calculate(self.cur_joint_pos)[3:6]
        # # print(l2_cur_pos)

        # start_pt = l2_cur_pos[:]
        # finish_pt = l2_cur_pos[:]
        # finish_pt[1] -= 0.05

        # print(f"start_pt: {start_pt}; finish_pt: {finish_pt}")

        # theta_ref[3:6] = self.ik.ikine_L1([finish_pt[0], finish_pt[1], finish_pt[2]], config=self.kinematic_scheme)
        # print(theta_ref[3:6])

        # time.sleep(5)


        # stand up
        theta_cur = theta_ref
        theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x, -self.ef_init_y, -self.robot_height,
                                             self.ef_init_x+self.cog_offset_x,  self.ef_init_y, -self.robot_height,
                                            -self.ef_init_x+self.cog_offset_x, -self.ef_init_y, -self.robot_height,
                                            -self.ef_init_x+self.cog_offset_x,  self.ef_init_y, -self.robot_height], config=self.kinematic_scheme)
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.5, 1/self.freq)
        self.take_position(theta_refs)

        # Finish

        self.finished = True

    # Put your functions below