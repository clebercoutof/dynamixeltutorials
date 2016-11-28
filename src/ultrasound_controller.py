#!/usr/bin/env python
import roslib

#roslib.load_manifest('my_dynamixel_tutorial')

import rospy
import actionlib
import sys
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, \
    FollowJointTrajectoryGoal
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
        goal.trajectory.joint_names = ['joint_3' + char, 'joint_4' + char]
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(3)
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)

def move(e):
     x = Joint('f_arm')
     value = e.value
     print('o valor e(cm):', value*1.296)
     x.move_joint([float(value)/40,float(value)/20])

def main():
    # Creating an interface Kit Object
    interfaceKit = InterfaceKit()
    # Opening Phidgets
    interfaceKit.openPhidget()
    # Waiting for attach
    interfaceKit.waitForAttach(10000)
    # Creating the Joint object. 'dynamixel_network' is a configuration file that groups all the servos involved
    # Whenever sensor value changes, move the servo.
    interfaceKit.setOnSensorChangeHandler(move)

    # Maintains the terminal open for the SensorChangeHandler call

    print("Press Enter to quit....")
    chr = sys.stdin.read(1)

    print("Closing...")

    interfaceKit.closePhidget()

    print("Done.")
    exit(0)
if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    main()
