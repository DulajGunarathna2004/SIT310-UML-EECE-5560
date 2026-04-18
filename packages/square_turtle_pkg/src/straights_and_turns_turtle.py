#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose


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

        self.goal_distance = 0.0
        self.goal_angle = 0.0
        self.goal_position = None

        self.start_distance = 0.0
        self.start_theta = 0.0

        self.mode = "idle"          

        self.linear_speed = 1.0
        self.angular_speed = 1.0

        self.distance_tolerance = 0.05
        self.angle_tolerance = 0.05
        self.position_tolerance = 0.10

        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def pose_callback(self, msg):
        self.pose = msg

    def dist_callback(self, msg):
        self.total_distance = msg.data

    def goal_distance_callback(self, msg):
        self.stop_all_goals()

        self.goal_distance = msg.data
        self.start_distance = self.total_distance

        if abs(self.goal_distance) < 1e-6:
            rospy.loginfo("Goal distance is 0, turtle will not move.")
            return

        self.mode = "distance"
        rospy.loginfo("Received goal distance: %.2f", self.goal_distance)

    def goal_angle_callback(self, msg):
        if self.pose is None:
            rospy.logwarn("Pose not received yet.")
            return

        self.stop_all_goals()

        self.goal_angle = msg.data
        self.start_theta = self.pose.theta

        if abs(self.goal_angle) < 1e-6:
            rospy.loginfo("Goal angle is 0, turtle will not rotate.")
            return

        self.mode = "angle"
        rospy.loginfo("Received goal angle: %.2f", self.goal_angle)

    def goal_position_callback(self, msg):
        if self.pose is None:
            rospy.logwarn("Pose not received yet.")
            return

        self.stop_all_goals()

        self.goal_position = msg
        self.mode = "position_turn"
        rospy.loginfo("Received goal position: x=%.2f, y=%.2f", msg.x, msg.y)

    def stop_all_goals(self):
        self.goal_distance = 0.0
        self.goal_angle = 0.0
        self.goal_position = None
        self.mode = "idle"
        self.publish_stop()

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self, event):
        if self.pose is None:
            return

        cmd = Twist()

        # DISTANCE MODE
        if self.mode == "distance":
            travelled = abs(self.total_distance - self.start_distance)
            target = abs(self.goal_distance)

            if travelled >= target - self.distance_tolerance:
                self.publish_stop()
                self.mode = "idle"
                rospy.loginfo("Reached goal distance")
                return

            if self.goal_distance > 0:
                cmd.linear.x = self.linear_speed
            else:
                cmd.linear.x = -self.linear_speed

            cmd.angular.z = 0.0

        # ANGLE MODE
        elif self.mode == "angle":
            turned = abs(self.normalize_angle(self.pose.theta - self.start_theta))
            target = abs(self.goal_angle)

            if turned >= target - self.angle_tolerance:
                self.publish_stop()
                self.mode = "idle"
                rospy.loginfo("Reached goal angle")
                return

            cmd.linear.x = 0.0
            if self.goal_angle > 0:
                cmd.angular.z = self.angular_speed      # counter-clockwise
            else:
                cmd.angular.z = -self.angular_speed     # clockwise

        # POSITION MODE - TURN FIRST
        elif self.mode == "position_turn":
            dx = self.goal_position.x - self.pose.x
            dy = self.goal_position.y - self.pose.y

            target_angle = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - self.pose.theta)

            if abs(angle_error) <= self.angle_tolerance:
                self.publish_stop()
                self.mode = "position_move"
                rospy.loginfo("Finished turning toward position")
                return

            cmd.linear.x = 0.0
            if angle_error > 0:
                cmd.angular.z = self.angular_speed
            else:
                cmd.angular.z = -self.angular_speed

        # POSITION MODE - THEN MOVE STRAIGHT
        elif self.mode == "position_move":
            dx = self.goal_position.x - self.pose.x
            dy = self.goal_position.y - self.pose.y
            distance_error = math.sqrt(dx * dx + dy * dy)

            if distance_error <= self.position_tolerance:
                self.publish_stop()
                self.mode = "idle"
                rospy.loginfo("Reached goal position")
                return

            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        # IDLE
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
