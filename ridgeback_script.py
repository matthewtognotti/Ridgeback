# /usr/bin/env python

# Author: Matt Tognotti
# Date: 9 Feburary 2023
# File: ridgeback_script.py

# Move_forward is a publisher node that publishes to the /ridgeback_velocity_controller/cmd_vel topic
# of type geometry_msgs/Twist

import rospy
from geometry_msgs.msg import Twist


def move_forward():

    print("\nStarting move_forward...")

    # Name of node is move_forward, anonmyous = True ensures the name is unique by adding extra numberrs
    rospy.init_node('move_forward', anonymous="True")

    # Publish to the ridgeback cmd_vel topic with type geometry_msgs/Twist
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Make sure ros is up and running
    if not rospy.is_shutdown():

        twist = Twist()
        twist.linear.x = 0.75
        twist.linear.y = 0.12
        twist.linear.z = 0.0
        twist.angular.x = 0.5
        twist.angular.y = 0.5
        twist.angular.z = 0.5

        # Print on screen and write to rosout
        rospy.loginfo(twist)
        # Publish to the topic cmd_vel
        pub.publish(twist)
        print("Done\n")


if __name__ == "__main__":
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass
