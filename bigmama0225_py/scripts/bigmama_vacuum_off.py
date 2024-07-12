#!/usr/bin/env python
import rospy, sys, numpy as np
import geometry_msgs.msg
import moveit_msgs.msg
from ur5_notebook.msg import Tracker
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty
def talker():
    track = rospy.Publisher('switch', Tracker, queue_size=1)
    rospy.init_node('vacuum_control',anonymous=True)   
    r = rospy.Rate(10) 
    sent = False  # Flag to ensure message is sent only once
    while not rospy.is_shutdown():
        if not sent:  # Check if the message has already been sent
            tracker = Tracker()
            tracker.flag2 = 0
            track.publish(tracker)  # Publish the 'off' message
            sent = True  # Set flag to True after sending the message
            if sent:
                print('message switch off has been sent')
        r.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:pass