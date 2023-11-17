import numpy as np
from zmp_controller.TrajectoryGenerator import create_multiple_trajectory
import time
from zmp_controller.ikine import IKineQuadruped


class Action4():
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
        for i in range(4):
            self.kp[3*i+0] -= 7
            self.kp[3*i+1] -= 7
            self.kp[3*i+2] -= 2

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


        self.ref_joint_pos = np.array(self.cur_joint_pos[:])
        time.sleep(0.1)

        # lay down
        offs_z = 0.1
        theta_ref = np.array(self.cur_joint_pos[:])
        theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x, -self.ef_init_y, -self.robot_height+offs_z,
                                       self.ef_init_x+self.cog_offset_x,  self.ef_init_y, -self.robot_height+offs_z,
                                      -self.ef_init_x+self.cog_offset_x, -self.ef_init_y, -self.robot_height+offs_z,
                                      -self.ef_init_x+self.cog_offset_x,  self.ef_init_y, -self.robot_height+offs_z], config=self.kinematic_scheme)
        theta_refs = create_multiple_trajectory(self.cur_joint_pos, theta_ref, 0.3, 1/self.freq)
        self.take_position(theta_refs)

        # print("1")
        # time.sleep(0.2)

        # go to back
        theta_cur = theta_ref[:]
        theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x, -self.ef_init_y+0.04, -self.robot_height-0.07,
                                       self.ef_init_x+self.cog_offset_x,  self.ef_init_y-0.04, -self.robot_height+offs_z,
                                      -self.ef_init_x+self.cog_offset_x, -self.ef_init_y+0.04, -self.robot_height-0.07,
                                      -self.ef_init_x+self.cog_offset_x,  self.ef_init_y-0.04, -self.robot_height+offs_z], config=self.kinematic_scheme)
        theta_refs = create_multiple_trajectory(self.cur_joint_pos, theta_ref, 0.3, 1/self.freq)
        self.take_position(theta_refs)

        # print("2")
        # print(theta_ref)
        # time.sleep(10.2)

        theta_cur = theta_ref[:]
        theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x, -self.ef_init_y+0.04, -self.robot_height-0.07,
                                       self.ef_init_x+self.cog_offset_x,  self.ef_init_y+0.04, -self.robot_height,
                                      -self.ef_init_x+self.cog_offset_x, -self.ef_init_y+0.04, -self.robot_height-0.07,
                                      -self.ef_init_x+self.cog_offset_x,  self.ef_init_y+0.04, -self.robot_height], config=self.kinematic_scheme)
        theta_ref[0] = -0.2
        theta_ref[1] = -2.0
        theta_ref[2] = 4.6
        theta_ref[6] = 0.2
        theta_ref[7] = -2.0
        theta_ref[8] = 4.6
        # print("set kp")
        # # self.ref_joint_kp[1] = 2
        # self.ref_joint_kp[2] = 2
        # # self.ref_joint_kp[7] = 2
        # self.ref_joint_kp[8] = 2

        theta_refs = create_multiple_trajectory(self.cur_joint_pos, theta_ref, 0.3, 1/self.freq)
        self.take_position(theta_refs)

        # print("3")
        # print(theta_ref)
        # time.sleep(10.2)

        theta_cur = theta_ref[:]
        theta_ref = [0, -1.57, 3.14, 
                      0, 1.57, -3.14, 
                      0, -1.57, 3.14, 
                      0, 1.57, -3.14]
        # self.ref_joint_kp[2] = 10
        # self.ref_joint_kp[8] = 10
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.3, 1/self.freq)
        self.take_position(theta_refs)

        # print("4")
        # time.sleep(0.2)

        theta_cur = theta_ref[:]
        theta_ref = [-0.8, -1.57, 3.14, 
                      0.5, 1.9, -4.5, 
                      0.8, -1.57, 3.14, 
                      -0.5, 1.9, -4.5]
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.2, 1/self.freq)
        self.take_position(theta_refs)

        # print("5")
        # time.sleep(3.2)

        theta_cur = theta_ref[:]
        theta_ref = [-1.0, -1.57, 3.14, 
                      0.36, 2.96, -5.81, 
                      1.0, -1.57, 3.14, 
                      -0.36, 2.96, -5.81]
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.15, 1/self.freq)
        self.take_position(theta_refs)

        # print("6")
        # time.sleep(15.5)

        theta_cur = theta_ref[:]
        theta_ref = [-1.1, -1.57, 3.14, 
                      -0.2, 1.14, -1.68, 
                      1.1, -1.57, 3.14, 
                      0.2, 1.14, -1.68]
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.2, 1/self.freq)
        self.take_position(theta_refs)

        # print("7")
        # time.sleep(10.2)

        # offs_x = 0.04
        # offs_z = 0.02
        # theta_ref = np.array(self.cur_joint_pos[:])
        # theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x+offs_x, -self.ef_init_y, -self.robot_height,
        #                                self.ef_init_x+self.cog_offset_x+offs_x,  self.ef_init_y, -self.robot_height,
        #                               -self.ef_init_x+self.cog_offset_x+offs_x, -self.ef_init_y, -self.robot_height+0.08,
        #                               -self.ef_init_x+self.cog_offset_x+offs_x,  self.ef_init_y, -self.robot_height+0.08], config=self.kinematic_scheme)
        # theta_refs = create_multiple_trajectory(self.cur_joint_pos, theta_ref, 0.5, 1/self.freq)
        # self.take_position(theta_refs)

        # time.sleep(1.0)        

        # theta_cur = theta_ref[:]
        # theta_ref[6] = 0
        # theta_ref[9] = 0
        # theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.5, 1/self.freq)
        # self.take_position(theta_refs)

        # time.sleep(1.0)

        # theta_cur = theta_ref[:]
        # theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x+offs_x, -self.ef_init_y, -self.robot_height+offs_z+0.03,
        #                                self.ef_init_x+self.cog_offset_x+offs_x,  self.ef_init_y, -self.robot_height+offs_z+0.03,
        #                               -self.ef_init_x+self.cog_offset_x+offs_x, -self.ef_init_y, -self.robot_height+0.08,
        #                               -self.ef_init_x+self.cog_offset_x+offs_x,  self.ef_init_y, -self.robot_height+0.08], config=self.kinematic_scheme)

        # theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.06, 1/self.freq)
        # self.take_position(theta_refs)
        

        # time.sleep(0.1)

        # theta_cur = theta_ref[:]
        # theta_ref[:3] = self.ik.ikine_R1([self.ef_init_x+self.cog_offset_x-0.04, -self.ef_init_y, -self.robot_height+offs_z-0.1], config=self.kinematic_scheme)
        # theta_ref[3:6] = self.ik.ikine_L1([self.ef_init_x+self.cog_offset_x-0.04,  self.ef_init_y, -self.robot_height+offs_z-0.1], config=self.kinematic_scheme)
        # theta_ref[7] = -1.7
        # theta_ref[10] = 1.7

        # theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.005, 1/self.freq)
        # print(theta_refs)
        # self.take_position(theta_refs)

        # theta_cur = theta_ref[:]
        # theta_ref[0] = 0
        # theta_ref[1] = -1.696
        # theta_ref[2] = 2.566
        # theta_ref[3] = 0
        # theta_ref[4] = 1.696
        # theta_ref[5] = -2.566
        # theta_ref[6] = 0
        # theta_ref[7] = -2.588
        # theta_ref[8] = 2.387
        # theta_ref[9] = 0
        # theta_ref[10] = 2.588
        # theta_ref[11] = -2.387
        # theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.2, 1/self.freq)
        # self.take_position(theta_refs)
        

        # time.sleep(1.5)

        # standup
        # go to initial position
        # theta_refs = np.array(self.cur_joint_pos[:])
        # if self.kinematic_scheme == 'm':
        #     theta_refs[2] = 3.14
        #     theta_refs[5] = -3.14
        #     theta_refs[8] = 3.14
        #     theta_refs[11] = -3.14
        # elif self.kinematic_scheme == 'x':
        #     theta_refs[2] = 3.14
        #     theta_refs[5] = -3.14
        #     theta_refs[8] = -3.14
        #     theta_refs[11] = 3.14
        # if self.kinematic_scheme == 'o':
        #     theta_refs[2] = -3.14
        #     theta_refs[5] = 3.14
        #     theta_refs[8] = 3.14
        #     theta_refs[11] = -3.14

        # theta_refs = create_multiple_trajectory(self.cur_joint_pos, theta_refs, 0.2, 1/self.freq)
        # self.take_position(theta_refs)

        # time.sleep(2)

        theta_cur = theta_ref[:]
        theta_ref = [0, -1.57, 3.14, 0, 1.57, -3.14, 0, -1.57, 3.14, 0, 1.57, -3.14]
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.25, 1/self.freq)
        self.take_position(theta_refs)
        
        # time.sleep(2)
        l = 0.09585
        abad_attach_y = 0.066
        angle = np.arccos((self.ef_init_y-abad_attach_y)/l)

        theta_cur = np.array(self.cur_joint_pos[:])
        theta_ref = np.array(self.cur_joint_pos[:])
        theta_ref[0] = angle
        theta_ref[6] = -angle
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.15, 1/self.freq)
        self.take_position(theta_refs)
        
        # time.sleep(1)
 
        theta_cur = np.array(self.cur_joint_pos[:])
        theta_ref = np.array(self.cur_joint_pos[:])
        theta_ref[3] = -angle
        theta_ref[9] = angle
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.15, 1/self.freq)
        self.take_position(theta_refs)
        
        theta_cur = np.array(self.cur_joint_pos[:])
        theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x, -self.ef_init_y, -self.robot_height,
                                             self.ef_init_x+self.cog_offset_x,  self.ef_init_y, -self.robot_height,
                                            -self.ef_init_x+self.cog_offset_x, -self.ef_init_y, -self.robot_height,
                                            -self.ef_init_x+self.cog_offset_x,  self.ef_init_y, -self.robot_height], config=self.kinematic_scheme)
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.3, 1/self.freq)
        self.take_position(theta_refs)

        # Finish

        self.finished = True

    # Put your functions below

    def kpkd_inc(self, t):
        ts = self.freq*t
        kp_inc = self.max_kp[:3]/ts
        kd_inc = self.max_kd[:3]/ts

        for i in range(3):
            if self.ref_joint_kp[i] < self.max_kp[i]:
                self.ref_joint_kp[i]   += kp_inc[i]
                self.ref_joint_kp[i+3] += kp_inc[i]
                self.ref_joint_kp[i+6] += kp_inc[i]
                self.ref_joint_kp[i+9] += kp_inc[i]

            if self.ref_joint_kd[i] < self.max_kd[i]:
                self.ref_joint_kd[i]   += kd_inc[i]
                self.ref_joint_kd[i+3] += kd_inc[i]
                self.ref_joint_kd[i+6] += kd_inc[i]
                self.ref_joint_kd[i+9] += kd_inc[i]

        time.sleep(1/self.freq)

    def kpkd_null(self):
        self.ref_joint_kp = np.array([0.0]*12)
        self.ref_joint_kd = np.array([0.0]*12)
        time.sleep(10/self.freq)

    def take_position(self, theta_refs):
        for i in range(len(theta_refs[0])):
            for j in range(12):
                self.ref_joint_pos[j] = theta_refs[j][i]

            time.sleep(1/self.freq)