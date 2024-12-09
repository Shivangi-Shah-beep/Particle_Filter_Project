import math
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import message_filters


class ImuandOdom():
    def __init__(self):
        # Subscribing to IMU and Odometry topics
        imu_sub = message_filters.Subscriber('/imu', Imu, queue_size=10)
        odom_sub = message_filters.Subscriber('/odom', Odometry, queue_size=10)

        # Synchronize IMU and Odometry messages
        ts = message_filters.ApproximateTimeSynchronizer([imu_sub, odom_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.callback)

        # Initialize variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_x = 0.0  # Linear velocity in X
        self.v_y = 0.0  # Linear velocity in Y
        self.last_time = rospy.Time.now()

    def callback(self, imu_msg, odom_msg):
        # Extract angular velocity from IMU (z-axis)
        #ang_vel = imu_msg.angular_velocity.z
        if imu_msg.angular_velocity.z==0:
            ang_vel= odom_msg.twist.twist.angular.z
            rospy.loginfo(f"The odometry angular velocity which was used is {ang_vel}")
        else:
            ang_vel = imu_msg.angular_velocity.z
            rospy.loginfo("Imu velocity is being used")
        # Extract linear acceleration from IMU
        accel_x = imu_msg.linear_acceleration.x
        accel_y = imu_msg.linear_acceleration.y

        # Extract linear velocity from odometry (fallback)
        odom_linear_vel = odom_msg.twist.twist.linear.x

        # Calculate time difference
        current_time = rospy.Time.now()
        delta_t = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Update velocity using acceleration
        self.v_x += accel_x * delta_t
        self.v_y += accel_y * delta_t

        # Use odometry linear velocity if IMU acceleration is unreliable
        if accel_x == 0.0 and accel_y == 0.0:
            self.v_x = odom_linear_vel
            self.v_y = 0.0  

        # Update pose using the motion model
        self.update_pose(self.v_x, self.v_y, ang_vel, delta_t)

    def update_pose(self, v_x, v_y, ang_vel, delta_t):
        # Apply motion model equations
        self.x += v_x * math.cos(self.theta) * delta_t - v_y * math.sin(self.theta) * delta_t
        self.y += v_x * math.sin(self.theta) * delta_t + v_y * math.cos(self.theta) * delta_t
        self.theta += ang_vel * delta_t

        # Normalize theta to [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Log updated pose
        rospy.loginfo(f"Updated Pose: x={self.x}, y={self.y}, theta={self.theta}")


def main():
    rospy.init_node('extract_imu_data_with_acceleration', anonymous=True)
    node = ImuandOdom()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
