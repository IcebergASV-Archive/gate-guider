#!/usr/bin/env python3


import rospy
import sensor_msgs.msg
from sensor_msgs.msg import LaserScan
from navigation.msg import Prop
import time


class ClassName(object):
    def __init__(self):
        print("NOTE: self is initializeing")
        self.laser=LaserScan()
        self.laser = rospy.wait_for_message('/rect_bot/laser/scan', LaserScan, timeout=5)
        self.prop=Prop()
        self.prop = rospy.wait_for_message('prop_angle_range', Prop, timeout=5)
        self.filled_out_prop=rospy.Publisher("/prop_closest_pnt", Prop, queue_size=1)
        print("NOTE: self has finished init")




    def publish_prop (self):

        print("NOTE: publish prop just got called")
        theta_1 = self.prop.theta_1
        print(theta_1)
        print("NOTE: just printed theta 1")
        theta_2 = self.prop.theta_2
        angle_max = self.laser.angle_max
        print(angle_max)
        print("angle max")
        #angle_incr = self.laser.angle_increment

        steps = 700 #(angle_max * 2) / angle_incr 

        index1 = (theta_1 + (angle_max - 1.570796327)) / angle_max
        index2 = (theta_2 + (angle_max - 1.570796327)) / angle_max

        #access laser snans between index 1 and index 2, return shortest
        ranges = self.laser.ranges

        #right now we are sweeping entire laser scan range
        print("NOTE: right after I set laser_ranges")
        #i=int(index1)
        i = 1
        closest_pnt = ranges[i]
        while (i<steps):
            if (ranges[i] < closest_pnt):
                closest_pnt = ranges[i]
                #min_angle_index = i
            i = i+1


        #closest_point_index = index(closest_point)
        print("NOTE: setting up publishing infos")
        prop_to_pub=Prop()
        prop_to_pub.prop_type=self.prop.prop_type
        prop_to_pub.theta_1=self.prop.theta_1
        prop_to_pub.theta_2=self.prop.theta_2
        prop_to_pub.closest_pnt_dist=closest_pnt

        self.filled_out_prop.publish(prop_to_pub)
        print("NOTE: end of publish_prop")

rospy.init_node("distance_finder_node")
r=rospy.Rate(10)
c=ClassName()
print("NOTE: just finished setting c=ClassName() in main part of code. ")

while not rospy.is_shutdown():
    print("NOTE: inside while loop")

    c.publish_prop()
    print("NOTE: inside c.publish prop")
    r.sleep()


