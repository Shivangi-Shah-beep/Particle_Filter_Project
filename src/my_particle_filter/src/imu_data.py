#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import message_filters

class PosePredictor:
    def __init__(self):
        # Subscribers
        imu_sub=message_filters.Subscriber('/imu', Imu)
        odom_sub=message_filters.Subscriber('/odom', Odometry)

        ts = message_filters.ApproximateTimeSynchronizer([imu_sub, odom_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.callback)

        # Publisher for predicted pose
        self.predicted_pose_pub = rospy.Publisher('/predicted_pose', PoseStamped, queue_size=10)

        # Current state
        self.last_time = None  # Last timestamp for integration
        self.predicted_pose = None  # Predicted pose (for publishing or debugging)
        self.odom_linear_velocity = None  # Linear velocity from odometry
        self.current_orientation = None  # Orientation from IMU

    def callback(self, imu_msg, odom_msg):
        # Extract linear velocity from odometry
        self.odom_linear_velocity = np.array([
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y,
            
        ])
        rospy.loginfo(f"Odometry linear velocity: {self.odom_linear_velocity}")

        if self.odom_linear_velocity is None:
            rospy.logwarn("Odometry linear velocity not available yet.")
            return

        # Extract the timestamp
        current_time = imu_msg.header.stamp.to_sec()
        if self.last_time is None:
            self.last_time = current_time
            return

        delta_t = current_time - self.last_time
        self.last_time = current_time

        # Extract IMU data
        angular_velocity_z = imu_msg.angular_velocity.z  # Angular velocity around Z-axis

        # Update pose using motion model
        self.predicted_pose = self.update_pose(angular_velocity_z, delta_t)

        # Publish the predicted pose
        self.publish_predicted_pose()

    def update_pose(self, angular_velocity_z, delta_t):
        # Initialize predicted pose if not already done
        if self.predicted_pose is None:
            self.predicted_pose = PoseStamped()
            self.predicted_pose.header.frame_id = "map"
            self.predicted_pose.pose.position.x = 0.0
            self.predicted_pose.pose.position.y = 0.0
            self.predicted_pose.pose.orientation.w = 1.0

        # Extract current pose and orientation
        x = self.predicted_pose.pose.position.x
        y = self.predicted_pose.pose.position.y

        quaternion = [
            self.predicted_pose.pose.orientation.x,
            self.predicted_pose.pose.orientation.y,
            self.predicted_pose.pose.orientation.z,
            self.predicted_pose.pose.orientation.w
        ]
        _, _, theta = tf.transformations.euler_from_quaternion(quaternion)

        # Integrate angular velocity to update orientation
        theta += angular_velocity_z * delta_t

        # Use odometry linear velocity for position update
        velocity_x = self.odom_linear_velocity[0]
        velocity_y = self.odom_linear_velocity[1]
        x += velocity_x * delta_t
        y += velocity_y * delta_t

        # Convert updated theta back to quaternion
        updated_quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)

        # Create and return updated pose
        predicted_pose = PoseStamped()
        predicted_pose.header.stamp = rospy.Time.now()
        predicted_pose.header.frame_id = "map"
        predicted_pose.pose.position.x = x
        predicted_pose.pose.position.y = y
        predicted_pose.pose.position.z = 0  # Keep Z constant
        predicted_pose.pose.orientation.x = updated_quaternion[0]
        predicted_pose.pose.orientation.y = updated_quaternion[1]
        predicted_pose.pose.orientation.z = updated_quaternion[2]
        predicted_pose.pose.orientation.w = updated_quaternion[3]

        return predicted_pose

    def publish_predicted_pose(self):
        if self.predicted_pose:
            self.predicted_pose_pub.publish(self.predicted_pose)
            rospy.loginfo(f"Published predicted pose: {self.predicted_pose.pose}")


def main():
    rospy.init_node('pose_predictor', anonymous=True)
    node = PosePredictor()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
