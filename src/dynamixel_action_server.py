#!/usr/bin/env python
import roslib

#roslib.load_manifest('my_dynamixel_tutorial')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, \
    FollowJointTrajectoryGoal
import time
from Phidgets.Devices.InterfaceKit import InterfaceKit

class Joint:
    def __init__(self, motor_name):
        # arm_name should be b_arm or f_arm
        self.name = motor_name
        self.jta = actionlib.SimpleActionClient('/' + self.name + '_controller/follow_joint_trajectory',
                                                FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

    def move_joint(self, angles):
        goal = FollowJointTrajectoryGoal()
        char = self.name[0]  # either 'f' or 'b'
        goal.trajectory.joint_names = ['joint_3' + char]
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(3)
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)


def main():
    arm = Joint('f_arm')
    interfaceKit = InterfaceKit()
    # Opening Phidgets
    interfaceKit.openPhidget()
    # Waiting for attach
    interfaceKit.waitForAttach(10000)
    while(True):
        time.sleep(1)
        x=interfaceKit.getSensorValue(7)
        value = float(x)/60
        if value>300 :
            print('A distancia e muito grande, o motor nao ira girar, valor(cm)=', x * 1.296)
            continue
        else:
            print('Distancia(cm)=', x*1.296)
            print('valor do giro=', value)
            arm.move_joint([value])


if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    main()
