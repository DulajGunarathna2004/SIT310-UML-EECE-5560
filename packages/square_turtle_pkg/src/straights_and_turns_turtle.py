#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def angle_diff(target, current):
    return normalize_angle(target - current)

class TurtlesimStraightsAndTurns:
    def __init__(self):
        
        # Distance variables
        self.current_distance = 0
        self.target_distance = 0
        self.initial_distance = 0
        self.distance_active = False
        self.move_forward = True

        # Pose
        self.current_pose = None

        # Angle variables
        self.angle_active = False
        self.target_angle = 0.0

        # Position variables
        self.position_active = False
        self.position_stage = "rotate"
        self.target_x = 0.0
        self.target_y = 0.0

        # Speeds
        self.linear_speed = 1.0
        self.angular_speed = 1.0

        # Tolerances
        self.angle_tolerance = 0.01
        self.position_tolerance = 0.05

        # Initialize node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Subscribers  
        rospy.Subscriber("/turtle_dist", Float64, self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)
        rospy.Subscriber("/goal_position", Point, self.goal_position_callback)

        # Publisher
        self.cmd_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Timer
        rospy.Timer(rospy.Duration(0.01), self.control_loop)

        rospy.loginfo("Initialized node!")
        rospy.spin()

    def pose_callback(self, msg):
        self.current_pose = msg

    def distance_callback(self, msg):
        self.current_distance = msg.data

    def goal_angle_callback(self, msg):
        if self.current_pose is None:
            return

        angle_input = msg.data

        if angle_input == 0:
            self.angle_active = False
            return

        self.target_angle = normalize_angle(self.current_pose.theta + angle_input)
        self.angle_active = True
        self.distance_active = False
        self.position_active = False

    def goal_distance_callback(self, msg):
        self.target_distance = abs(msg.data)
        self.move_forward = msg.data >= 0
        self.initial_distance = self.current_distance

        if self.target_distance == 0:
            self.distance_active = False
        else:
            self.distance_active = True

        self.angle_active = False
        self.position_active = False

    def goal_position_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.position_active = True
        self.position_stage = "rotate"

        self.distance_active = False
        self.angle_active = False

    def control_loop(self, event):
        cmd = Twist()

        # POSITION CONTROL
        if self.position_active:
            if self.current_pose is None:
                return

            dx = self.target_x - self.current_pose.x
            dy = self.target_y - self.current_pose.y
            distance_error = math.sqrt(dx * dx + dy * dy)

            if distance_error <= self.position_tolerance:
                self.position_active = False
            else:
                desired_angle = math.atan2(dy, dx)
                angle_error = angle_diff(desired_angle, self.current_pose.theta)

                if self.position_stage == "rotate" and abs(angle_error) > self.angle_tolerance:
                    cmd.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
                else:
                    self.position_stage = "move"
                    if abs(angle_error) > self.angle_tolerance:
                        cmd.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
                    cmd.linear.x = self.linear_speed

        # ANGLE CONTROL
        elif self.angle_active and self.current_pose is not None:
            angle_error = angle_diff(self.target_angle, self.current_pose.theta)

            if abs(angle_error) <= self.angle_tolerance:
                self.angle_active = False
            else:
                cmd.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed

        # DISTANCE CONTROL
        elif self.distance_active:
            travelled = self.current_distance - self.initial_distance

            if travelled >= self.target_distance:
                self.distance_active = False
            else:
                cmd.linear.x = self.linear_speed if self.move_forward else -self.linear_speed

        # STOP
        if not (self.distance_active or self.angle_active or self.position_active):
            cmd = Twist()

        self.cmd_publisher.publish(cmd)


if __name__ == '__main__': 
    try: 
        TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException: 
        pass
