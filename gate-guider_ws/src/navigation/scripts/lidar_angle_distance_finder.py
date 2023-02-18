#!/usr/bin/env python


import rospy

def talker():
    pub = rospy.Publisher('chatter', queue_size=10)



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass