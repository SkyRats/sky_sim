#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publish_message():
    # Initialize the ROS node
    rospy.init_node('my_publisher', anonymous=True)

    # Create a publisher for the topic '/my_topic' with message type 'String'
    pub = rospy.Publisher('/my_topic', String, queue_size=10)

    # Set the publishing rate (in Hz)
    rate = rospy.Rate(1)  # 1 Hz

    # Main loop
    while not rospy.is_shutdown():
        # Create a message
        message = "Hello, ROS!"

        # Publish the message
        pub.publish(message)

        # Sleep to maintain the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
