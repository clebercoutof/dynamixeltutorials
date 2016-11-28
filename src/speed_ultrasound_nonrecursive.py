#!/usr/bin/env python

#Packages necessary for ROS use
import rospy
from std_msgs.msg import Float64
#Sytem package
import sys,select,os
#Package necessary for PHIDGETS use
from Phidgets.Devices.InterfaceKit import InterfaceKit

class Motor:
    def __init__(self, servo1 , servo2):
        #Initiate our node
        rospy.init_node('ultrasound_wheel_controller')
        #The motor name wich will be used to create our publisher, in this case, pan1 or pan3
        self.servo1 = servo1
        self.servo2 = servo2
        self.servo1pub = rospy.Publisher('/'+self.servo1+'_controller/command', Float64, queue_size=10)
        self.servo2pub = rospy.Publisher('/' + self.servo2 + '_controller/command', Float64, queue_size=10)
    #This function displays the sensor value and spin the motor with an angular velocity inversely proportional
    def spin(self,value):
        #Removes the sensor offset
        value = value - 58
        # Calculates the distance
        dist= x = value * 1.428 + 20
        print('DISTANCIA(cm)=', dist)
        spin_speed = 0.02*float(value)
        if dist < 20:
            self.servo1pub.publish(0)
            self.servo2pub.publish(0)
        else:
            self.servo1pub.publish(spin_speed)
            self.servo2pub.publish(spin_speed/2)

    #This method stop the servos
    def stop(self):
        self.servo1pub.publish(0)
        self.servo2pub.publish(0)

def main():
    #Creating our Motor object
    controller = Motor('pan1', 'pan3')
    # Creating an interface Kit Object
    interfaceKit = InterfaceKit()
    # Opening Phidgets
    interfaceKit.openPhidget()
    # Waiting for attach
    interfaceKit.waitForAttach(10000)
    print("Press Enter to quit....")
    #Main Loop
    while(True):
        #Stops the loop when you press ENTER
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = raw_input()
            print("Closing...")
            # Closes phidgets
            interfaceKit.closePhidget()
            # Stop the servos
            controller.stop()
            print("Done.")
            exit(0)
            break
        else:
            #Sleeps for half second
            rospy.sleep(.5)
            #Gets the sensor value
            x = interfaceKit.getSensorValue(7)
            #Spin the motors
            controller.spin(x)


if __name__ == '__main__':
    main()