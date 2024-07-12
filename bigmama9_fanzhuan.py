#!/usr/bin/env python

import rosbag
import rospy

def publish_bag_content(trajectory_fanzhuan, publish_rate=0.1):
    """
    Publishes the content of a given bag file.
    
    :param bag_file_name: Path to the bag file to be published.
    :param publish_rate: Rate at which to publish messages.
    """
    # Initialize a node for the script
    rospy.init_node('bag_publisher', anonymous=True)

    # Create a dictionary to hold publishers for each topic
    publishers = {}

    with rosbag.Bag(trajectory_fanzhuan, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            # If a publisher for this topic does not exist, create one
            if topic not in publishers:
                publishers[topic] = rospy.Publisher(topic, type(msg), queue_size=10)
            
            # Publish the message
            publishers[topic].publish(msg)
            
            # Sleep according to the specified rate
            rospy.sleep(1.0 / publish_rate)

if __name__ == '__main__':
    BAG_FILE_NAME = '/home/hyh/catkin_ws/src/trajectory_fanzhuan.bag'  # Replace with the path to your bag file
    publish_rate = 0.3  # Adjust the rate as needed
    try:
        publish_bag_content(BAG_FILE_NAME, publish_rate)
    except rospy.ROSInterruptException:
        pass
