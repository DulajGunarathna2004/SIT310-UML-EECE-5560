#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose


class StraightAndTurnController:
    def __init__(self):
        rospy.init_node('straight_and_turn_controller', anonymous=True)

        # Publisher
        self.cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        rospy.Subscriber('/turtle_dist', Float64, self.dist_callback)
        rospy.Subscriber('/goal_distance', Float64, self.goal_distance_callback)
        rospy.Subscriber('/goal_angle', Float64, self.goal_angle_callback)
        rospy.Subscriber('/goal_position', Point, self.goal_position_callback)

        # Current turtle data
        self.pose = None
        self.total_distance = 0.0

        # Goal data
        self.goal_distance = None
        self.goal_angle = None
        self.goal_position = None

        # Starting values for current goal
        self.start_distance = 0.0
        self.start_theta = 0.0

        # State flags
        self.moving_straight = False
        self.turning = False
        self.go_to_position = False
        self.position_stage = None   # "turn" or "move"

        # Speeds
        self.linear_speed = 1.0
        self.angular_speed = 1.0

        # Tolerances
        self.distance_tolerance = 0.05
        self.angle_tolerance = 0.05
        self.position_tolerance = 0.1

        # Timer callback
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def pose_callback(self, msg):
        self.pose = msg

    def dist_callback(self, msg):
        self.total_distance = msg.data

    def goal_distance_callback(self, msg):
        self.clear_all_goals()

        self.goal_distance = msg.data

        if abs(self.goal_distance) < 1e-6:
            rospy.loginfo("Goal distance is 0. Turtle will not move.")
            return

        self.start_distance = self.total_distance
        self.moving_straight = True
        rospy.loginfo("Received goal distance: %.2f", self.goal_distance)

    def goal_angle_callback(self, msg):
        if self.pose is None:
            rospy.logwarn("Pose not received yet. Cannot start angle goal.")
            return

        self.clear_all_goals()

        # Task uses turtle angle values, so keep this in radians
        self.goal_angle = msg.data

        if abs(self.goal_angle) < 1e-6:
            rospy.loginfo("Goal angle is 0. Turtle will not rotate.")
            return

        self.start_theta = self.pose.theta
        self.turning = True
        rospy.loginfo("Received goal angle: %.2f radians", self.goal_angle)

    def goal_position_callback(self, msg):
        if self.pose is None:
            rospy.logwarn("Pose not received yet. Cannot start position goal.")
            return

        self.clear_all_goals()

        self.goal_position = msg
        self.go_to_position = True
        self.position_stage = "turn"

        rospy.loginfo("Received goal position: x=%.2f, y=%.2f", msg.x, msg.y)

    def clear_all_goals(self):
        self.goal_distance = None
        self.goal_angle = None
        self.goal_position = None

        self.moving_straight = False
        self.turning = False
        self.go_to_position = False
        self.position_stage = None

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_turtle(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def control_loop(self, event):
        if self.pose is None:
            return

        cmd = Twist()

        # -------------------------------------------------
        # 1. DISTANCE GOAL
        # -------------------------------------------------
        if self.moving_straight and self.goal_distance is not None:
            travelled = abs(self.total_distance - self.start_distance)
            target = abs(self.goal_distance)

            if travelled < (target - self.distance_tolerance):
                if self.goal_distance > 0:
                    cmd.linear.x = self.linear_speed
                else:
                    cmd.linear.x = -self.linear_speed
                cmd.angular.z = 0.0
            else:
                self.stop_turtle()
                self.moving_straight = False
                rospy.loginfo("Reached goal distance")

        # -------------------------------------------------
        # 2. ANGLE GOAL
        # -------------------------------------------------
        elif self.turning and self.goal_angle is not None:
            turned = abs(self.normalize_angle(self.pose.theta - self.start_theta))
            target = abs(self.goal_angle)

            if turned < (target - self.angle_tolerance):
                cmd.linear.x = 0.0
                if self.goal_angle > 0:
                    cmd.angular.z = self.angular_speed      # CCW
                else:
                    cmd.angular.z = -self.angular_speed     # CW
            else:
                self.stop_turtle()
                self.turning = False
                rospy.loginfo("Reached goal angle")

        # -------------------------------------------------
        # 3. POSITION GOAL
        # -------------------------------------------------
        elif self.go_to_position and self.goal_position is not None:
            dx = self.goal_position.x - self.pose.x
            dy = self.goal_position.y - self.pose.y

            distance_to_goal = math.sqrt(dx * dx + dy * dy)
            target_angle = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - self.pose.theta)

            # Stage 1: rotate toward target
            if self.position_stage == "turn":
                if abs(angle_error) > self.angle_tolerance:
                    cmd.linear.x = 0.0
                    if angle_error > 0:
                        cmd.angular.z = self.angular_speed
                    else:
                        cmd.angular.z = -self.angular_speed
                else:
                    self.stop_turtle()
                    self.position_stage = "move"
                    rospy.loginfo("Finished turning toward position goal")

            # Stage 2: move straight
            elif self.position_stage == "move":
                if distance_to_goal > self.position_tolerance:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = 0.0
                else:
                    self.stop_turtle()
                    self.go_to_position = False
                    self.position_stage = None
                    rospy.loginfo("Reached goal position")

        # -------------------------------------------------
        # 4. NO ACTIVE GOAL
        # -------------------------------------------------
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
