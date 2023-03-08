#!/usr/bin/env python

import Jetson.GPIO as GPIO
import time
import rospy

from std_msgs.msg import String

SERVO_PIN = 33

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0.0)


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    gimbal_pitch = data.data

def gimbal():
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('gimbal', anonymous=True)
    rate = rospy.Rate(10)
    
    rospy.Subscriber('gimba_pitch', String, callback)
    

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()

        for t_high in range(60, 80):
            pwm.ChangeDutyCycle(t_high/10.0)
            print(t_high/10.0)
            time.sleep(1)

        pwm.ChangeDutyCycle(6.0)
        print("6")
        time.sleep(2.0)

        pwm.ChangeDutyCycle(0.0)
        print("0")
        time.sleep(2.0)

        pwm.ChangeDutyCycle(7.0)
        print("7")
        time.sleep(2.0)

    rospy.spin()

        

if __name__ == '__main__':
    try:
        gimbal()
    except rospy.ROSInterruptException:
        pwm.stop()
        GPIO.cleanup()
        pass



 
