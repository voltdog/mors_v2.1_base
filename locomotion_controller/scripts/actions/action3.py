import numpy as np
from zmp_controller.TrajectoryGenerator import create_multiple_trajectory
import time
from zmp_controller.ikine import IKineQuadruped


class Action3():
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
            # self.kp[3*i+0] -= 7
            # self.kp[3*i+1] -= 7
            self.kp[3*i+2] += 2

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
    
    def take_position(self, theta_refs):
        for i in range(len(theta_refs[0])):
            for j in range(12):
                self.ref_joint_pos[j] = theta_refs[j][i]

            time.sleep(1/(8*self.freq))

    def execute(self):
        self.finished = False

        # Put your code here
        self.ref_joint_kp = np.array(self.kp, dtype=float)
        self.ref_joint_kd = np.array(self.kd, dtype=float)

        cog_point_x = 0.04
        cog_point_y = 0.04
        r1_point = 0.08
        sit_point = 0.02
        theta_cur = np.array(self.cur_joint_pos[:])
        theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x+cog_point_x, -self.ef_init_y-cog_point_y, -self.robot_height-sit_point+r1_point,
                                       self.ef_init_x+self.cog_offset_x+cog_point_x,  self.ef_init_y-cog_point_y, -self.robot_height-sit_point,
                                      -self.ef_init_x+self.cog_offset_x+cog_point_x, -self.ef_init_y-cog_point_y, -self.robot_height+sit_point,
                                      -self.ef_init_x+self.cog_offset_x+cog_point_x,  self.ef_init_y-cog_point_y, -self.robot_height+sit_point], config=self.kinematic_scheme)
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.25, 1/self.freq)
        self.take_position(theta_refs)

        # time.sleep(2)
        theta_cur = theta_ref[:]
        theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x+0.23, -self.ef_init_y-cog_point_y, -self.robot_height-sit_point+r1_point+0.07,
                                       self.ef_init_x+self.cog_offset_x+cog_point_x,  self.ef_init_y-cog_point_y, -self.robot_height-sit_point,
                                      -self.ef_init_x+self.cog_offset_x+cog_point_x, -self.ef_init_y-cog_point_y, -self.robot_height+sit_point,
                                      -self.ef_init_x+self.cog_offset_x+cog_point_x,  self.ef_init_y-cog_point_y, -self.robot_height+sit_point], config=self.kinematic_scheme)
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.3, 1/self.freq)
        self.take_position(theta_refs)

        for _ in range(4):

            # time.sleep(0.1)
            theta_cur = theta_ref[:]
            theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x+0.23, -self.ef_init_y-cog_point_y, -self.robot_height-sit_point+r1_point+0.09,
                                           self.ef_init_x+self.cog_offset_x+cog_point_x,  self.ef_init_y-cog_point_y, -self.robot_height-sit_point,
                                          -self.ef_init_x+self.cog_offset_x+cog_point_x, -self.ef_init_y-cog_point_y, -self.robot_height+sit_point,
                                          -self.ef_init_x+self.cog_offset_x+cog_point_x,  self.ef_init_y-cog_point_y, -self.robot_height+sit_point], config=self.kinematic_scheme)
            theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.1, 1/self.freq)
            self.take_position(theta_refs)

            # time.sleep(0.1)
            theta_cur = theta_ref[:]
            theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x+0.23, -self.ef_init_y-cog_point_y, -self.robot_height-sit_point+r1_point+0.04,
                                           self.ef_init_x+self.cog_offset_x+cog_point_x,  self.ef_init_y-cog_point_y, -self.robot_height-sit_point,
                                          -self.ef_init_x+self.cog_offset_x+cog_point_x, -self.ef_init_y-cog_point_y, -self.robot_height+sit_point,
                                          -self.ef_init_x+self.cog_offset_x+cog_point_x,  self.ef_init_y-cog_point_y, -self.robot_height+sit_point], config=self.kinematic_scheme)
            theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.1, 1/self.freq)
            self.take_position(theta_refs)


        

        # time.sleep(0.1)
        theta_cur = theta_ref[:]
        theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x+cog_point_x, -self.ef_init_y-cog_point_y, -self.robot_height-sit_point+r1_point,
                                       self.ef_init_x+self.cog_offset_x+cog_point_x,  self.ef_init_y-cog_point_y, -self.robot_height-sit_point,
                                      -self.ef_init_x+self.cog_offset_x+cog_point_x, -self.ef_init_y-cog_point_y, -self.robot_height+sit_point,
                                      -self.ef_init_x+self.cog_offset_x+cog_point_x,  self.ef_init_y-cog_point_y, -self.robot_height+sit_point], config=self.kinematic_scheme)
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.25, 1/self.freq)
        self.take_position(theta_refs)

        # time.sleep(0.5)

        theta_cur = theta_ref[:]
        theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x, -self.ef_init_y, -self.robot_height,
                                       self.ef_init_x+self.cog_offset_x,  self.ef_init_y, -self.robot_height,
                                      -self.ef_init_x+self.cog_offset_x, -self.ef_init_y, -self.robot_height,
                                      -self.ef_init_x+self.cog_offset_x,  self.ef_init_y, -self.robot_height], config=self.kinematic_scheme)
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.35, 1/self.freq)
        self.take_position(theta_refs)
        
        # Finish

        self.finished = True

    # Put your functions below
    