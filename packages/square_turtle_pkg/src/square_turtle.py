#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def draw_square():
    rospy.init_node('square_turtle', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    move = Twist()
    rate = rospy.Rate(1)  # 1 loop per second

    while not rospy.is_shutdown():
        for i in range(4):
            # Move forward
            move.linear.x = 2.0
            move.angular.z = 0.0
            pub.publish(move)
            rospy.sleep(2)

            # Turn 90 degrees
            move.linear.x = 0.0
            move.angular.z = 1.57
            pub.publish(move)
            rospy.sleep(1)

if __name__ == '__main__':
    try:
        draw_square()
    except rospy.ROSInterruptException:
        pass
