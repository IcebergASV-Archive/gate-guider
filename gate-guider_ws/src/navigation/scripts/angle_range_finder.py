#!/usr/bin/env python3


import rospy
from navigation.msg import Prop




def angle_range_finder():
    pub = rospy.Publisher('prop_angle_range', Prop)
    rospy.init_node('angle_range_finder_node', anonymous=True)
    rate = rospy.Rate(10) #10Hz
    msg = Prop()
    msg.prop_type = "red_marker"
    msg.theta_1 = 177
    msg.theta_2 = 4

    while not rospy.is_shutdown():
       rospy.loginfo(msg)
       pub.publish(msg)
       rate.sleep()



if __name__ == '__main__':
    try:
        angle_range_finder()
    except rospy.ROSInterruptException:
        pass



