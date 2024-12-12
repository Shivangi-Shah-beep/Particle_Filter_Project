#!/usr/bin/env python3
import rospy
import numpy as np
import tf
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
import math

class LidarMap:
    def __init__(self):
        # Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=10)
        rospy.Subscriber('/particles', PoseArray, self.particle_callback, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=10)

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Internal storage
        self.resolution = None
        self.map_width = None
        self.map_height = None
        self.map_x = None
        self.map_y = None
        self.occupancy_grid = None
        self.particles = []

        # Transform parameters (static from tf_echo)
        self.T_ro = np.array([
            [0, 0, 1, -0.064],  # Translation: -0.064 along X
            [-1, 0, 0, 0.0],    # Translation: 0.0 along Y
            [0, -1, 0, 0.122],  # Translation: 0.122 along Z
            [0, 0, 0, 1]
        ])

        # Gaussian noise standard deviation
        self.sigma = 0.1

    def map_callback(self, msg):
        self.resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_x = msg.info.origin.position.x
        self.map_y = msg.info.origin.position.y
        self.occupancy_grid = np.array(msg.data).reshape((self.map_height, self.map_width))

        rospy.loginfo(f"Map loaded: {self.map_width}x{self.map_height} cells, resolution: {self.resolution}m")
        rospy.loginfo(f"Map origin: ({self.map_x}, {self.map_y})")

    def particle_callback(self, msg):
        self.particles = []
        for pose in msg.poses:
            particle_x = pose.position.x
            particle_y = pose.position.y
            quaternion = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ]
            _, _, particle_theta = tf.transformations.euler_from_quaternion(quaternion)
            self.particles.append((particle_x, particle_y, particle_theta))
        
        #rospy.loginfo(f"Received {len(self.particles)} particles.")

    def simulate_lidar(self, x, y, theta, max_range, angle_increment, num_beams):
        """
        Simulate the LiDAR ranges for a given particle using raycasting.
        """
        ranges = []
        for i in range(num_beams):
            angle = theta + (i - num_beams // 2) * angle_increment
            for r in np.arange(0, max_range, self.resolution):
                rx = x + r * math.cos(angle)
                ry = y + r * math.sin(angle)
                cell_x, cell_y = self.world_to_grid(rx, ry)
                if 0 <= cell_x < self.map_width and 0 <= cell_y < self.map_height:
                    if self.occupancy_grid[cell_y, cell_x] > 0:  # Occupied cell
                        ranges.append(r)
                        break
            else:
                ranges.append(max_range)
        return ranges

    def compute_particle_weights(self, laser_scan):
        """
        Compute the weights of particles based on the error between the actual LiDAR scan and the simulated scan.
        """
        if not self.particles or not self.map_initialized():
            rospy.logwarn("No particles or map data available. Skipping weight computation.")
            return

        # Store actual ranges and parameters from the LiDAR scan
        actual_ranges = np.array(laser_scan.ranges)
        angle_increment = laser_scan.angle_increment
        max_range = laser_scan.range_max
        num_beams = len(actual_ranges)

        weights = []
        for particle_x, particle_y, particle_theta in self.particles:
            T_mr = np.array([
                [np.cos(particle_theta), -np.sin(particle_theta), 0, particle_x],
                [np.sin(particle_theta), np.cos(particle_theta), 0, particle_y],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
            T_mo = np.dot(T_mr, self.T_ro)

            # Simulate LiDAR scan for the particle
            simulated_ranges = self.simulate_lidar(particle_x, particle_y, particle_theta, max_range, angle_increment, num_beams)

            # Compute errors and particle weight
            errors = actual_ranges - simulated_ranges
            gaussian_weights = np.exp(-0.5 * (errors ** 2) / (self.sigma ** 2))
            particle_weight = np.sum(gaussian_weights)
            weights.append(particle_weight)

        # Normalize weights
        weights = np.array(weights)
        weights /= np.sum(weights)
        rospy.loginfo(f"Normalized weights: {weights}")
        return weights

    def world_to_grid(self, world_x, world_y):
        """
        Convert world coordinates to grid indices.
        """
        cell_x = int((world_x - self.map_x) / self.resolution)
        cell_y = int((world_y - self.map_y) / self.resolution)
        return cell_x, cell_y

    def map_initialized(self):
        """
        Check if the map is fully initialized.
        """
        return self.occupancy_grid is not None

    def lidar_callback(self, laser_scan):
        """
        Process LiDAR data, transform for each particle, and compute particle weights.
        """
        if not self.particles:
            rospy.logwarn("No particles available for transformation.")
            return

        # Transform each LiDAR point and compute particle weights
        weights = self.compute_particle_weights(laser_scan)
        rospy.loginfo(f"Particle weights computed: {weights}")


def main():
    rospy.init_node('lidar_map_processor', anonymous=True)
    node = LidarMap()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
