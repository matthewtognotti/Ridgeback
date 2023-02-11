#!/usr/bin/env python

# Author: Matt Tognotti
# Date: 9 Feburary 2023
# File: ridgeback_script.py

# Move_ridgeback  is a  node that publishes to the /cmd_vel topic
# of type geometry_msgs/Twist

# This code serves as an example of a publisher and is not necessarily useful on its own 

import rospy
from geometry_msgs.msg import Twist
# Twist expresses velocity broken into its linear and angular parts

def move_ridgeback():

    # Name of node is move_ridgeback, anonmyous = True ensures the name is unique by adding extra numberrs
    rospy.init_node('move_ridgeback', anonymous="True")

    # Create publisher to the cmd_vel topic with type geometry_msgs/Twist
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Note: when publishing to a topic once, sometimes the message is not receieved by the subscriber if
    # there is not enough time between sending the message and the subscriber registering with the new publisher
    # Therefore, we need to wait until the connection between this publisher and the subsriber is made
    while pub.get_num_connections() < 1:
        pass

    # Make sure ros is up and running
    if not rospy.is_shutdown():

        twist = Twist()
        twist.linear.x = 0.75
        twist.linear.y = 0.12
        twist.linear.z = 0.0
        twist.angular.x = 0.5
        twist.angular.y = 0.5
        twist.angular.z = 0.5

        print("\nStarting move_forward...")

        # Print on screen and write to rosout
        rospy.loginfo(twist)
        # Publish to the topic cmd_vel
        pub.publish(twist)
        print("Done!\n")


if __name__ == "__main__":
    try:
        move_ridgeback()
    except rospy.ROSInterruptException:
        pass
