#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
import math
from duckietown_msgs.msg import WheelEncoderStamped

class OdomCalc:
    def __init__(self):
        # Initialize the pose of the robot
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0

        # Robot Position Parameters
        self.left_ticks = None
        self.right_ticks = None

        self.previous_left_ticks = None
        self.previous_right_ticks = None

        rospy.init_node("odometry_calculator", anonymous=True)

        #Publisher Pose
        self.pose_pub = rospy.Publisher("pose", Pose2D, queue_size=10)

        #Subscribers Odometry one per wheel encoder
        self.left_wheel_sub = rospy.Subscriber("left_wheel_encoder_driver_node/tick", WheelEncoderStamped, self.left_wheel_callback)
        self.right_wheel_sub = rospy.Subscriber("right_wheel_encoder_driver_node/tick", WheelEncoderStamped, self.right_wheel_callback)

        #set excecution rate
        self.rate = rospy.Rate(10) # 10hz
        rospy.loginfo("Odometry calculator node initialized.")

    def left_wheel_callback(self, msg):
        # Process left wheel encoder data
        rospy.loginfo("Left wheel encoder data received: %s", msg.data)

        # Odometry calculation logic here
        self.left_ticks = msg.data

    def right_wheel_callback(self, msg):
        # Process right wheel encoder data
        rospy.loginfo("Right wheel encoder data received: %s", msg.data)
        
        # Odometry calculation logic here
        self.right_ticks = msg.data

    def calculate_odometry(self):
        # Use these same counts throughout this update.
        current_left = self.left_ticks
        current_right = self.right_ticks

        if current_left is None or current_right is None:
            return

        # First measurements establish the starting counts.
        if self.previous_left_ticks is None or self.previous_right_ticks is None:
            self.previous_left_ticks = current_left
            self.previous_right_ticks = current_right
            self.pose_pub.publish(self.pose)
            return

        # 1. Calculate new ticks since the previous update.
        delta_left = current_left - self.previous_left_ticks
        delta_right = current_right - self.previous_right_ticks

        # Save exactly the counts used above.
        self.previous_left_ticks = current_left
        self.previous_right_ticks = current_right

        # 2. Convert ticks into wheel travel.
        wheel_radius = 0.0318       # Required lab test radius, meters
        ticks_per_revolution = 135
        baseline = 0.1             # Required lab test wheel spacing, meters

        distance_per_tick = (
            2.0 * math.pi * wheel_radius / ticks_per_revolution
        )

        left_distance = delta_left * distance_per_tick
        right_distance = delta_right * distance_per_tick

        # 3. Calculate center travel and change in heading.
        average_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / baseline

        # 4. Update position using the midpoint heading.
        heading_midpoint = self.pose.theta + delta_theta / 2.0

        self.pose.x += average_distance * math.cos(heading_midpoint)
        self.pose.y += average_distance * math.sin(heading_midpoint)
        self.pose.theta += delta_theta

        # Keep heading between -pi and pi.
        self.pose.theta = (
            (self.pose.theta + math.pi) % (2.0 * math.pi) - math.pi
        )

        # 5. Publish the estimated pose.
        self.pose_pub.publish(self.pose)

    def run(self):
        while not rospy.is_shutdown():
            self.calculate_odometry()
            self.rate.sleep()   

if __name__ == "__main__":
    try:
        odom_calc = OdomCalc()
        odom_calc.run()
    except rospy.ROSInterruptException:
        pass