#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
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
        rospy.Subscriber('/goal_position', Point, self.goal_position_callback)

        self.pose = None
        self.total_distance = 0.0

        self.goal_distance = None
        self.goal_angle = None
        self.goal_position = None

        self.start_distance = None
        self.start_theta = None

        self.moving_straight = False
        self.turning = False
        self.moving_to_position = False
        self.position_stage = None   # "turn" or "move"

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
        self.moving_to_position = False

        if self.goal_distance == 0:
            self.moving_straight = False
            rospy.loginfo("Goal distance is 0, turtle will not move.")
        else:
            rospy.loginfo("Received goal distance: %f", self.goal_distance)

    def goal_angle_callback(self, msg):
        if self.pose is not None:
            self.goal_angle = msg.data   # input in radians
            self.start_theta = self.pose.theta
            self.turning = True
            self.moving_straight = False
            self.moving_to_position = False

            if self.goal_angle == 0:
                self.turning = False
                rospy.loginfo("Goal angle is 0, turtle will not rotate.")
            else:
                rospy.loginfo("Received goal angle: %f radians", self.goal_angle)

    def goal_position_callback(self, msg):
        if self.pose is not None:
            self.goal_position = msg
            self.moving_to_position = True
            self.moving_straight = False
            self.turning = False
            self.position_stage = "turn"
            rospy.loginfo("Received goal position: x=%f, y=%f", msg.x, msg.y)

    def angle_difference(self, a, b):
        diff = a - b
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def control_loop(self, event):
        cmd = Twist()

        if self.moving_straight and self.goal_distance is not None:
            travelled = self.total_distance - self.start_distance

            if abs(travelled) < abs(self.goal_distance):
                if self.goal_distance > 0:
                    cmd.linear.x = 1.0
                else:
                    cmd.linear.x = -1.0
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.moving_straight = False
                rospy.loginfo("Reached goal distance")

        elif self.turning and self.goal_angle is not None and self.pose is not None:
            turned = self.angle_difference(self.pose.theta, self.start_theta)

            if abs(turned) < abs(self.goal_angle):
                cmd.linear.x = 0.0
                if self.goal_angle > 0:
                    cmd.angular.z = 1.0      # counter-clockwise
                else:
                    cmd.angular.z = -1.0     # clockwise
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.turning = False
                rospy.loginfo("Reached goal angle")

        elif self.moving_to_position and self.goal_position is not None and self.pose is not None:
            dx = self.goal_position.x - self.pose.x
            dy = self.goal_position.y - self.pose.y

            target_angle = math.atan2(dy, dx)
            angle_error = self.angle_difference(target_angle, self.pose.theta)
            distance_error = math.sqrt(dx * dx + dy * dy)

            if self.position_stage == "turn":
                if abs(angle_error) > 0.05:
                    cmd.linear.x = 0.0
                    if angle_error > 0:
                        cmd.angular.z = 1.0
                    else:
                        cmd.angular.z = -1.0
                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.position_stage = "move"

            elif self.position_stage == "move":
                if distance_error > 0.1:
                    cmd.linear.x = 1.0
                    cmd.angular.z = 0.0
                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.moving_to_position = False
                    self.position_stage = None
                    rospy.loginfo("Reached goal position")

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
