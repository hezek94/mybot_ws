#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf import transformations
import math
#glaobal variables

actual_yaw_=0
state_=0
position_=Point()

desire_position_=Point()

desire_position_.x=-3
desire_position_.y=7
desire_position_.z=0
#param

yaw_precision_=math.pi/90
dist_precision=0.3

pub=None

def callback(msg):
    global position_, actual_yaw_
# declaring position from the actual position and the yaw angle of the robot in enviroment
    position_=msg.pose.pose.position
    quat1=msg.pose.pose.orientation.x
    quat2=msg.pose.pose.orientation.y
    quat3=msg.pose.pose.orientation.z
    quat4=msg.pose.pose.orientation.w

    quaternion_=(quat1,quat2,quat3,quat4)
    euler_parm=transformations.euler_from_quaternion(quaternion_)
    actual_yaw_=euler_parm[2]

#declaring the function for pathplaning 

def correct_yaw(destination):
    #get the postion in x and y then find the yaw angle using yaw
    global actual_yaw_, pub, yaw_precision_, state_
# we call actual yaw and precision with state to make the necessary adjustment incase we enter some errors
    desired_yaw=math.atan2(destination.y-position_.y, destination.x-position_.x)
    err_yaw=desired_yaw -actual_yaw_
    
    twist_msg=Twist()
    
    
        
    if math.fabs(err_yaw)>yaw_precision_:
        twist_msg.angular.z=-0.3 if err_yaw>0 else 0.3 #terniary operator that return a ccw or cw
    
    pub.publish(twist_msg)

    if math.fabs(err_yaw)>yaw_precision_:
        print ('yaw error: [%s]' %err_yaw)
        change_state(1)

        


def move_straigth_ahead(destination):
    #get the postion in x and y then find the yaw angle using yaw
    global actual_yaw_, pub, yaw_precision_, state_
# we call actual yaw and precision with state to make the necessary adjustment incase we enter some errors
    desired_yaw=math.atan(destination.y-position_.y, destination.x-position_.x)
    err_yaw=desired_yaw -actual_yaw_
    err_position= math.sqrt(pow(destination.y-position_.y,2)+pow(destination.x-position_.x,2))

    if err_position>dist_precision:
        twist_msg=Twist()
        twist_msg.linear.x=0.3
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]' % err_position)
        change_state(2)

    if math.fabs(err_yaw)>yaw_precision_:
        print ('yaw error: [%s]' %err_yaw)
        change_state(0)


def change_state(state):
    global state_
    state_=state

    print('state change : [%s]' % state_) 

def finish():
    twist_msg=Twist()
    twist_msg.linear.x=0
    twist_msg.angular.z=0
    pub.publish(twist_msg)


    def main():
        global pub
        rospy.init_node('to_goal')

        pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        sub=rospy.Subscriber('/odom', Odometry, callback)
        rate=rospy.Rate(20)

        while not rospy.is_shutdown():
            if state_==0:
                correct_yaw(desire_position_)
            elif state_==1:
                move_straigth_ahead(desire_position_)
            elif state_==2:
                finish()
                pass
            else:
                rospy.logerr('state unknown')

            pass
        rate.sleep()
    if __name__ == '__main__':
        main()









