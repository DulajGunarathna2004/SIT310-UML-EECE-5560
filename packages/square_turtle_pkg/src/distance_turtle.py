#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time 

class DistanceReader:
    def __init__(self):
        
        # Initialize the node
        rospy.init_node('turtlesim_distance_node', anonymous=True)

        # Initialize subscriber: input the topic name, message type and callback signature  
        rospy.Subscriber("/turtle1/pose", Pose, self.callback)

        # Initialize publisher: input the topic name, message type and msg queue size
        self.distance_publisher = rospy.Publisher('/turtle_dist', Float64, queue_size=10)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized node!")

        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0
        
        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    # Whenever a message is received from the specified subscriber, this function will be called
    def callback(self, msg):
        rospy.loginfo("Turtle Position: %s %s", msg.x, msg.y)

        # first pose message: just store the starting position
        if self.prev_x is None and self.prev_y is None:
            self.prev_x = msg.x
            self.prev_y = msg.y
            return

        # calculate small movement distance
        dx = msg.x - self.prev_x
        dy = msg.y - self.prev_y
        distance = (dx ** 2 + dy ** 2) ** 0.5

        # add to total distance
        self.total_distance += distance

        # update previous position
        self.prev_x = msg.x
        self.prev_y = msg.y

        # publish total distance
        self.distance_publisher.publish(self.total_distance)

        # print total distance
        rospy.loginfo("Total Distance Travelled: %f", self.total_distance)

if __name__ == '__main__': 

    try: 
        distance_reader_class_instance = DistanceReader()
    except rospy.ROSInterruptException: 
        pass
