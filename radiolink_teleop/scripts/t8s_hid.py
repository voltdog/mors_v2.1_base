import rospy
import time
from threading import Thread
import numpy as np

from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray, Pose

import radiolink_teleop.joystick as j
from mors.srv import QuadrupedCmd, QuadrupedCmdResponse

STANDUP = 1
LAY_DOWN = 2
GIVE_HAND = 3
SIDE_ROLL = 4
SIT_DOWN = 5


# call robot_mode service
def set_mode_client(mode):
    rospy.wait_for_service('robot_mode')
    try:
        set_mode = rospy.ServiceProxy('robot_mode', QuadrupedCmd)
        resp = set_mode(mode)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# call robot_action service
def set_action_client(action):
    rospy.wait_for_service('robot_action')
    try:
        set_action = rospy.ServiceProxy('robot_action', QuadrupedCmd)
        resp = set_action(action)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def set_stride_height_client(height):
    rospy.wait_for_service('stride_height')
    try:
        set_height = rospy.ServiceProxy('stride_height', QuadrupedCmd)
        resp = set_height(height)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    # ros things
    rospy.init_node("radiolink_teleop")

    frequency = rospy.get_param("/general/frequency", 40)

    cmd_vel = rospy.get_param("~topics/cmd_vel", "cmd_vel")
    cmd_pose = rospy.get_param("~topics/cmd_pose", "cmd_pose")
    cmd_ef_pos = rospy.get_param("~topics/cmd_ef_pose", "ef_position/command")
    cmd_joint_pos = rospy.get_param("~topics/cmd_joint_pos", "joint_group_position_controller/command")
    joy_status = rospy.get_param("~topics/status", "joy_status")

    max_speed_x = rospy.get_param("~gait/max_speed_x", 0.3)
    max_speed_y = rospy.get_param("~gait/max_speed_y", 0.2)
    max_speed_z = rospy.get_param("~gait/max_speed_z", 0.8)
    stride_height = rospy.get_param("~gait/stride_height", 0.04)

    walk_mode = rospy.get_param("~modes/walk", 0)
    ef_mode = rospy.get_param("~modes/ef", 1)
    body_mode = rospy.get_param("~modes/body", 2)
    joint_mode = rospy.get_param("~modes/joint", 3)

    max_body_x = rospy.get_param("~pose/max_lin_x", 0.1)
    max_body_y = rospy.get_param("~pose/max_lin_y", 0.1)
    max_body_z = rospy.get_param("~pose/max_lin_z", 0.1)
    min_body_z = rospy.get_param("~pose/min_lin_z", 0.1)
    max_body_wx = rospy.get_param("~pose/max_angle_x", 0.1)
    max_body_wy = rospy.get_param("~pose/max_angle_y", 0.1)
    max_body_wz = rospy.get_param("~pose/max_angle_z", 0.1)

    rate = rospy.Rate(frequency)

    cmd_vel_pub = rospy.Publisher(cmd_vel, Twist, queue_size=10)
    cmd_pose_pub = rospy.Publisher(cmd_pose, Twist, queue_size=10)
    cmd_ef_pos_pub = rospy.Publisher(cmd_ef_pos, PoseArray, queue_size=10)
    cmd_joint_pos_pub = rospy.Publisher(cmd_joint_pos, JointTrajectoryPoint, queue_size=10)
    status_pub = rospy.Publisher(joy_status, Bool, queue_size=10)

    cmd_vel_msg = Twist()
    cmd_pose_msg = Twist()
    cmd_ef_msg = PoseArray()
    cmd_one_ef_msg = Pose()
    cmd_joint_msg = JointTrajectoryPoint()

    # services data
    mode = 0
    mode_prev = 0
    action = 0
    action_prev = 0

    # connection status data
    joy_fault = False
    rec_fault = False
    connected = False

    # thread for handling joystick data
    joy = j.Joystick()
    th = Thread(target=joy.loop_read, args=())
    th.daemon = True
    th.start()

    is_standup = 0

    set_mode_client(mode)
    set_stride_height_client(stride_height)
    stride_height_set = False

    joy_fault_first = True
    rec_fault_first = True
    just_connected = True

    rospy.loginfo("radiolink_teleop started!")

    while not rospy.is_shutdown():
        status = joy.get_joy_status()
        if status == j.CONNECTED:
            if just_connected:
                set_stride_height_client(stride_height)
                set_mode_client(mode)
                just_connected = False

            joy.play_loop()
            joy_fault = 0
            rec_fault = 0
            t8s = joy.get_joy_data()
            # print(t8s)
            # if R2 in middle position
            if t8s[j.R2] == 1 and is_standup == 1:
                mode = walk_mode

                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.linear.y = 0
                cmd_vel_msg.angular.z = 0

                cmd_pose_msg.linear.z = 0
                cmd_pose_msg.angular.x = 0
                cmd_pose_msg.angular.y = 0
                cmd_pose_msg.angular.z = 0

            # if R2 in down position
            elif t8s[j.R2] == 0 and is_standup == 1:
                mode = walk_mode

                cmd_vel_msg.linear.x = t8s[j.LY]*max_speed_x
                cmd_vel_msg.linear.y = -t8s[j.LX]*max_speed_y
                cmd_vel_msg.angular.z = -t8s[j.RX]*max_speed_z

                if abs(cmd_vel_msg.linear.x) < 0.1:
                    cmd_vel_msg.linear.x = 0 
                if abs(cmd_vel_msg.linear.y) < 0.05:
                    cmd_vel_msg.linear.y = 0
                if abs(cmd_vel_msg.angular.z) < 0.07:
                    cmd_vel_msg.angular.z = 0
                else:
                    cmd_vel_msg.linear.y = 0
                    if 0.0 <= cmd_vel_msg.linear.x  < 0.1:
                        cmd_vel_msg.linear.x  = 0.1
                    elif cmd_vel_msg.linear.x > 0.6:
                        cmd_vel_msg.linear.x = 0.6

            # if R2 in up position
            elif t8s[j.R2] == 2  and is_standup == 1:
                mode = body_mode

                if t8s[j.LY] >= 0:
                    cmd_pose_msg.linear.z = t8s[j.LY] * max_body_z
                else:
                    cmd_pose_msg.linear.z = t8s[j.LY] * min_body_z

                cmd_pose_msg.angular.x = t8s[j.LX] * max_body_wx * (1 - np.abs(t8s[j.LY]))
                cmd_pose_msg.angular.y = t8s[j.RY] * max_body_wy
                cmd_pose_msg.angular.z = t8s[j.RX] * max_body_wz

                if abs(cmd_pose_msg.linear.z) < 0.005:
                    cmd_pose_msg.linear.z = 0

                if abs(cmd_pose_msg.angular.x) < 0.04:
                    cmd_pose_msg.angular.x = 0

                if abs(cmd_pose_msg.angular.y) < 0.04:
                    cmd_pose_msg.angular.y = 0

                if abs(cmd_pose_msg.angular.z) < 0.04:
                    cmd_pose_msg.angular.z = 0

            # whether should call robot_mode or robot_action services 
            if mode != mode_prev:
                set_mode_client(mode)
            mode_prev = mode

            if t8s[j.R1] == 0:
                action = 0
            elif t8s[j.R1] == 1:
                action = 1

            if t8s[j.L2] == 0:
                action = GIVE_HAND # give hand
            elif t8s[j.L2] == 2:
                action = SIT_DOWN # sit down

            if t8s[j.L1] <= 22000:
                action = SIDE_ROLL

            # print(t8s[j.L2])

            if action == 1 and action_prev == 0  and is_standup == 0:
                if set_action_client(STANDUP) == 1:
                    is_standup = 1
                    rospy.loginfo("standup")
            elif action == 1 and action_prev == 0  and is_standup == 1:
                if set_action_client(LAY_DOWN) == 1:
                    is_standup = 0
                    rospy.loginfo("laydown")
            elif action == GIVE_HAND and action_prev == 0 and is_standup == 1:
                if set_action_client(GIVE_HAND) == 1:
                    rospy.loginfo("finished giving hand")
            elif action == SIT_DOWN and action_prev == 0 and is_standup == 1:
                if set_action_client(SIT_DOWN) == 1:
                    rospy.loginfo("finished sitting down")
            elif action == SIDE_ROLL and action_prev == 0 and is_standup == 1:
                if set_action_client(SIDE_ROLL) == 1:
                    rospy.loginfo("finished rolling")
            action_prev = action

            # publish command messages
            cmd_vel_pub.publish(cmd_vel_msg)
            cmd_pose_pub.publish(cmd_pose_msg)

            joy_fault_first = True
            rec_fault_first = True


        # if there are problems with joystick connection
        elif status == j.RECEIVER_ERROR:
            if joy_fault_first:
                rospy.loginfo("Radiolink receiver disconnected. Trying to connect...")
                joy_fault_first = False
            joy.pause_loop()
            rec_fault = 1
            just_connected = True
            joy.connect()
            time.sleep(0.5)
        elif status == j.JOYSTICK_ERROR:
            if rec_fault_first:
                rospy.loginfo("Radiolink joystick disconnected. Please turn it on, connect the receiver or use other devices to control robot...")
                rec_fault_first = False
            joy.pause_loop()
            joy_fault = 1
            just_connected = True
            joy.connect()
            joy.read_data()
            time.sleep(0.5)

        

        status_pub.publish(not (bool(joy_fault) or bool(rec_fault)))
        rate.sleep()

