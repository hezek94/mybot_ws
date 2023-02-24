#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    #using the minimum value from the list 
    space_covered = [
        min(min(msg.ranges[0:143]), 30),
        min(min(msg.ranges[144:287]), 30),
        min(min(msg.ranges[288:431]), 30),
        min(min(msg.ranges[432:575]), 30),
        min(min(msg.ranges[576:713]), 30),]
    rospy.loginfo(space_covered)

def main():
    rospy.init_node("laser_data")
    rospy.Subscriber('/mybot/laser/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    main()

