#!/usr/bin/env python
#Package needed to interact with ROS
import rospy
from std_msgs.msg import Float64
import sys
#Package needed to use phidgets
from Phidgets.Devices.InterfaceKit import InterfaceKit

class Motor:
    def __init__(self, servo1 , servo2):
        # Initiate our node
        rospy.init_node('ultrasound_wheel_controller')
        self.rate= rospy.Rate(60)
        #The motor name wich will be used to create our publisher, in this case, pan1 or pan3
        self.servo1 = servo1
        self.servo2 = servo2
        self.servo1pub = rospy.Publisher('/'+self.servo1+'_controller/command', Float64, queue_size=10)
        self.servo2pub = rospy.Publisher('/' + self.servo2 + '_controller/command', Float64, queue_size=10)

    def spin(self,e):
        value = e.value
        #Removes the sensor offset
        value = value - 58
        #Calculates the distance
        dist = value * 1.428 + 20
        print('DISTANCIA(cm)=', dist)
        spin_speed = 0.02*float(value)
        if dist <= 25:
            self.servo1pub.publish(0)
            self.servo2pub.publish(0)
            self.rate.sleep()
        else:
            self.servo1pub.publish(spin_speed)
            self.servo2pub.publish(spin_speed/2)
            self.rate.sleep()

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
    # Whenever sensor value changes, spin the servo.
    interfaceKit.setOnSensorChangeHandler(controller.spin)
    # Maintains the terminal open for the SensorChangeHandler call
    print("Press Enter to quit....")
    chr = sys.stdin.read(1)
    print("Closing...")
    #Closes phidgets
    interfaceKit.closePhidget()
    #Stop the servos
    controller.stop()
    print("Done.")
    exit(0)

if __name__ == '__main__':
    main()
