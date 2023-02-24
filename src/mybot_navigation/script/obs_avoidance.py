#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pb = None
def callback(msg):
    space_t={
        'rgt': min(min(msg.ranges[0:143]), 30),
        'frgt': min(min(msg.ranges[144:287]), 30),
        'frnt':min(min(msg.ranges[288:431]), 30),
        'flt' :min(min(msg.ranges[432:575]), 30),
        'lft' :min(min(msg.ranges[576:713]), 30),
    }
    react_tn(space_t)


def react_tn(space_t):
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    if space_t['frnt'] > 1 and space_t['flt'] > 1 and space_t['frgt'] > 1:
        state_description = 'case 1 - nothing'
        linear_x = 0.6
        angular_z = 0
    elif space_t['frnt'] < 1 and space_t['flt'] > 1 and space_t['frgt'] > 1:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = -0.3
    elif space_t['frnt'] > 1 and space_t['flt'] > 1 and space_t['frgt'] < 1:
        state_description = 'case 3 - fright'
        linear_x = 0
        angular_z = -0.3
    elif space_t['frnt'] > 1 and space_t['flt'] < 1 and space_t['frgt'] > 1:
        state_description = 'case 4 - fleft'
        linear_x = 0
        angular_z = 0.3
    elif space_t['frnt'] < 1 and space_t['flt'] > 1 and space_t['frgt'] < 1:
        state_description = 'case 5 - front and fright'
        linear_x = 0
        angular_z = -0.3
    elif space_t['frnt'] < 1 and space_t['flt'] < 1 and space_t['frgt'] > 1:
        state_description = 'case 6 - front and fleft'
        linear_x = 0
        angular_z = 0.3
    elif space_t['frnt'] < 1 and space_t['flt'] < 1 and space_t['frgt'] < 1:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0
        angular_z = -0.3
    elif space_t['frnt'] > 1 and space_t['flt'] < 1 and space_t['frgt'] < 1:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0
        angular_z = -0.3
    else:
        state_description = 'unknown case'
        rospy.loginfo(space_t)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pb.publish(msg)


def main():
    global pb
    rospy.init_node('obs_avoidance')
    pb= rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub= rospy.Subscriber('/mybot/laser/scan', LaserScan, callback)

    rospy.spin()

if __name__=="__main__":
    main()