#!/usr/bin/env python3
import rospy
import os
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time

emotion_pub = rospy.Publisher('quori/face_generator_emotion', Float64MultiArray, queue_size=10)
movement_pub = rospy.Publisher('quori/joint_trajectory_controller/command', JointTrajectory, queue_size=10)

rospy.init_node('behavior_publisher', anonymous=True)
rate = rospy.Rate(10) # 10hz

def neutral():
    #Pick a position close to neutral
    point = JointTrajectoryPoint()
    traj = JointTrajectory()
    a = 0.1
    b = 2
    point.time_from_start = rospy.Duration((b-a) * np.random.random_sample() + a)
    traj.joint_names = ["r_shoulder_pitch", "r_shoulder_roll", "l_shoulder_pitch", "l_shoulder_roll", "waist_pitch", "turret"]
    neutral_positions = np.array([0, -1.1, 0, -1.1, 0, 0])
    a = -0.1
    b = 0.1
    point.positions = (neutral_positions + (b-a) * np.random.random_sample((6,)) + a).tolist()
    traj.points=[point]
    movement_pub.publish(traj)

    neutral_emotion = np.zeros((6,))
    a = 0
    b = 0.15
    emotion_to_send = Float64MultiArray()  # the data to be sent, initialise the array
    emotion_to_send.data = (neutral_emotion + (b-a) * np.random.random_sample((6,)) + a).tolist()
    emotion_pub.publish(emotion_to_send)
    
def happy(intensity, duration):
    #Pick a position as a percentage of ideal happy + noise for movement

    #Torso backward
    torso_max = -0.21*0.2
    actual_torso = intensity * torso_max

    #Arms symmetrical moving to Y position
    arm_sides = np.array([0, -1.1])
    arm_y = np.array([-np.pi/2*0.2, -0.95*0.2])
    distance = arm_y - arm_sides
    distance_to_move = distance*intensity
    actual_arm = (arm_y - arm_sides) / (distance) * distance_to_move

    #Create trajectory to end position and back
    traj = JointTrajectory()
    traj.joint_names = ["r_shoulder_pitch", "r_shoulder_roll", "l_shoulder_pitch", "l_shoulder_roll", "waist_pitch", "turret"]

    #End point of movement takes half the time
    point_1 = JointTrajectoryPoint()
    point_1.time_from_start = rospy.Duration(duration / 2)
    point_1.positions = np.array([actual_arm[0], actual_arm[1], actual_arm[0], actual_arm[1], actual_torso, 0]).tolist()
    traj.points=[point_1]
    movement_pub.publish(traj)

    print('Positions', point_1.positions)
    time.sleep(duration/2)

    #Happy emotion
    emotion_arr = [intensity, 0, 0, 0, 0, 0]
    print('Emotion', emotion_arr)
    emotion_to_send = Float64MultiArray()  # the data to be sent, initialise the array
    emotion_to_send.data = emotion_arr
    emotion_pub.publish(emotion_to_send)

    #Back to neutral takes other half

    traj = JointTrajectory()
    traj.joint_names = ["r_shoulder_pitch", "r_shoulder_roll", "l_shoulder_pitch", "l_shoulder_roll", "waist_pitch", "turret"]

    point_2 = JointTrajectoryPoint()
    point_2.time_from_start = rospy.Duration(duration / 2)
    point_2.positions = [0, -1.1, 0, -1.1, 0, 0]
    traj.points=[point_2]
    movement_pub.publish(traj)

    #Neutral emotion
    emotion_arr = [0, 0, 0, 0, 0, 0]
    emotion_to_send = Float64MultiArray()  # the data to be sent, initialise the array
    emotion_to_send.data = emotion_arr
    emotion_pub.publish(emotion_to_send)


    time.sleep(duration/2.0)

if __name__ == '__main__':

    iters = 0
    while iters < 5:
        neutral()
        a = 1
        b = 5
        time.sleep((b-a)*np.random.random_sample() + a)
        iters += 1
        print('Neutral', iters)

    iters = 0
    while iters < 10:
        happy(iters/10.0, 8)
        iters += 1
        print(iters)

    # iters = 0
    # while iters < 10:
    #     neutral()
    #     a = 1
    #     b = 5
    #     time.sleep((b-a)*np.random.random_sample() + a)
    #     iters += 1


# def talker():
    
    

#     #joy, sadness, anger, disgust, fear, surprise
#     emotion = [1, 0, 0.5, 0, 0.25, 0.5]
#     data_to_send = Float64MultiArray()  # the data to be sent, initialise the array
#     data_to_send.data = emotion
#     while not rospy.is_shutdown():
#         emotion_pub.publish(data_to_send)
#         rate.sleep()

