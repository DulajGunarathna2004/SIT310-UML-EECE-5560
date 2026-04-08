#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

class StraightAndTurnController:
    def __init__(self):
        rospy.init_node('straight_and_turn_controller', anonymous=True)

        self.cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        rospy.Subscriber('/turtle_dist', Float64, self.dist_callback)
        rospy.Subscriber('/goal_distance', Float64, self.goal_distance_callback)
        rospy.Subscriber('/goal_angle', Float64, self.goal_angle_callback)

        self.pose = None
        self.total_distance = 0.0

        self.goal_distance = None
        self.goal_angle = None

        self.start_distance = None
        self.start_theta = None

        self.moving_straight = False
        self.turning = False

        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def pose_callback(self, msg):
        self.pose = msg

    def dist_callback(self, msg):
        self.total_distance = msg.data

    def goal_distance_callback(self, msg):
        self.goal_distance = msg.data
        self.start_distance = self.total_distance
        self.moving_straight = True
        self.turning = False
        rospy.loginfo("Received goal distance: %f", self.goal_distance)

    def goal_angle_callback(self, msg):
        if self.pose is not None:
            self.goal_angle = math.radians(msg.data)   # input in degrees
            self.start_theta = self.pose.theta
            self.turning = True
            self.moving_straight = False
            rospy.loginfo("Received goal angle: %f degrees", msg.data)

    def angle_difference(self, a, b):
        diff = a - b
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return abs(diff)

    def control_loop(self, event):
        cmd = Twist()

        if self.moving_straight and self.goal_distance is not None:
            travelled = self.total_distance - self.start_distance

            if travelled < self.goal_distance:
                cmd.linear.x = 1.0
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.moving_straight = False
                rospy.loginfo("Reached goal distance")

        elif self.turning and self.goal_angle is not None and self.pose is not None:
            turned = self.angle_difference(self.pose.theta, self.start_theta)

            if turned < abs(self.goal_angle):
                cmd.linear.x = 0.0
                cmd.angular.z = 1.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.turning = False
                rospy.loginfo("Reached goal angle")

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        StraightAndTurnController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass