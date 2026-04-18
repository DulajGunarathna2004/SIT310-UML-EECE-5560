#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

def normalize_angle(angle):
    # Wrap angle to [-pi, pi]
    return math.atan2(math.sin(angle), math.cos(angle))
    # return angle

def angle_diff(target, current):
    # Smallest difference from current to target with wraparound
    return normalize_angle(target - current)

class TurtlesimStraightsAndTurns:
    def __init__(self):
        
        # Initialize class variables
        self.last_distance = 0
        self.goal_distance = 0
        self.start_distance = 0
        self.dist_goal_active = False
        self.forward_movement = True
        self.pose = None
        self.angle_goal_active = False
        self.angle_target = 0.0
        self.position_goal_active = False
        self.position_mode = "rotate"
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.linear_speed = 1.0
        self.angular_speed = 1.0
        self.angle_tolerance = 0.01
        self.position_tolerance = 0.05

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers  
        rospy.Subscriber("/turtle_dist", Float64,self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64,self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64,self.goal_distance_callback)
        rospy.Subscriber("/turtle1/pose", Pose,self.pose_callback)
        rospy.Subscriber("/goal_position", Point, self.goal_position_callback)

        # Initialize publishers
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Initialize a timer. The timer callback will act as our main function
        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized node!")
        
        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    def pose_callback(self,msg):
        self.pose = msg

    def distance_callback(self,msg):
        self.last_distance = msg.data

    def goal_angle_callback(self,msg):
        if self.pose is None:
            return

        delta = msg.data
        if delta == 0:
            self.angle_goal_active = False
            return

        self.angle_target = normalize_angle(self.pose.theta + delta)
        self.angle_goal_active = True
        self.dist_goal_active = False
        self.position_goal_active = False

    def goal_distance_callback(self,msg):
        ########## YOUR CODE GOES HERE ##########
        # Set goal_distance, dist_goal_active and forward_movement variables here
        self.goal_distance = abs(msg.data)
        self.forward_movement = msg.data >= 0
        self.start_distance = self.last_distance  # reset odometer at start of new goal

        if self.goal_distance == 0:
            self.dist_goal_active = False
        else:
            self.dist_goal_active = True

        self.angle_goal_active = False
        self.position_goal_active = False

        ###########################################

    def goal_position_callback(self,msg):
        # Accept (x, y) goal for straight-then-turn behavior
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.position_goal_active = True
        self.position_mode = "rotate"
        self.dist_goal_active = False
        self.angle_goal_active = False

    def timer_callback(self,msg):
        ########## YOUR CODE GOES HERE ##########
        # If a goal is active, first check if the goal is reached (it's OK if the goal is not perfectly reached)
        # Then publish a cmd_vel message
        twist = Twist()

        if self.position_goal_active:
            if self.pose is None:
                return

            dx = self.goal_x - self.pose.x
            dy = self.goal_y - self.pose.y
            distance_to_goal = math.sqrt(dx * dx + dy * dy)

            if distance_to_goal <= self.position_tolerance:
                self.position_goal_active = False
            else:
                desired_heading = math.atan2(dy, dx)
                heading_error = angle_diff(desired_heading, self.pose.theta)

                if self.position_mode == "rotate" and abs(heading_error) > self.angle_tolerance:
                    twist.angular.z = self.angular_speed if heading_error > 0 else -self.angular_speed
                else:
                    self.position_mode = "move"
                    if abs(heading_error) > self.angle_tolerance:
                        twist.angular.z = self.angular_speed if heading_error > 0 else -self.angular_speed
                    twist.linear.x = self.linear_speed

        elif self.angle_goal_active and self.pose is not None:
            angle_error = angle_diff(self.angle_target, self.pose.theta)
            if abs(angle_error) <= self.angle_tolerance:
                self.angle_goal_active = False
            else:
                twist.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed

        elif self.dist_goal_active:
            traveled = self.last_distance - self.start_distance
            if traveled >= self.goal_distance:
                self.dist_goal_active = False
            else:
                twist.linear.x = self.linear_speed if self.forward_movement else -self.linear_speed

        if not (self.dist_goal_active or self.angle_goal_active or self.position_goal_active):
            twist = Twist()  # ensure stop when no goal is active

        self.velocity_publisher.publish(twist)

        ###########################################

if __name__ == '__main__': 

    try: 
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException: 
        pass
