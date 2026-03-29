

import rospy
from geometry_msgs.msg import Twist

def move_square():
    rospy.init_node('square_turtle_node', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 times per second
    vel = Twist()

    rospy.sleep(1)

    while not rospy.is_shutdown():
        # Move forward
        vel.linear.x = 2.0
        vel.angular.z = 0.0
        for _ in range(20):   # move for 2 seconds
            pub.publish(vel)
            rate.sleep()

        # Stop a little
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        for _ in range(5):
            pub.publish(vel)
            rate.sleep()

        # Turn left 90 degrees
        vel.linear.x = 0.0
        vel.angular.z = 1.57
        for _ in range(10):   # turn for about 1 second
            pub.publish(vel)
            rate.sleep()

        # Stop a little
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        for _ in range(5):
            pub.publish(vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.loginfo("Turtles are great at drawing squares!")
        move_square()
    except rospy.ROSInterruptException:
        pass