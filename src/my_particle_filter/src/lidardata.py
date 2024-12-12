#!/usr/bin/env python3

import numpy as np
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
import math


class LidarData:
    def __init__(self):
        # Map info
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=10)
        # Particle info
        rospy.Subscriber('/particles', PoseArray, self.particle_callback, queue_size=10)
        # Lidar info
        self.scan_pub = rospy.Publisher('/simulated_scan', LaserScan, queue_size=10)

        # Storage
        self.resolution = None
        self.map_width = None
        self.map_height = None
        self.map_x = None
        self.map_y = None
        self.occupancy_grid = None
        self.particles = []

    # For map
    def map_callback(self, msg):
        self.resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_x = msg.info.origin.position.x
        self.map_y = msg.info.origin.position.y
        self.occupancy_grid = np.array(msg.data).reshape((self.map_height, self.map_width))

        rospy.loginfo(f"Map loaded: {self.map_width}x{self.map_height} cells, resolution: {self.resolution}m")
        rospy.loginfo(f"Map origin: ({self.map_x}, {self.map_y})")
    
    def world_to_grid(self, world_x, world_y):
        cell_x = int((world_x - self.map_x) / self.resolution)
        cell_y = int((world_y - self.map_y) / self.resolution)
        return cell_x, cell_y

    def grid_to_world(self, cell_x, cell_y):
        world_x = self.map_x + (cell_x * self.resolution)
        world_y = self.map_y + (cell_y * self.resolution)
        return world_x, world_y

    def is_occupied(self, world_x, world_y):
        cell_x, cell_y = self.world_to_grid(world_x, world_y)
        if 0 <= cell_x < self.map_width and 0 <= cell_y < self.map_height:
            return self.occupancy_grid[cell_y, cell_x] > 0  # Assuming >0 means occupied
        else:
            return False  # Out of bounds

    def particle_callback(self, msg):
        self.particles = []

        # Iterate through all poses in the PoseArray
        for pose in msg.poses:
            particle_x = pose.position.x
            particle_y = pose.position.y
            _, _, particle_yaw = tf.transformations.euler_from_quaternion(
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            )
            self.particles.append((particle_x, particle_y, particle_yaw))

        rospy.loginfo(f"Received {len(self.particles)} particles.")

        # Simulate LiDAR for all particles
        self.simulate_lidar_for_particles()

    def raycast(self, particle_pose, max_range=10.0, fov=2 * math.pi, angular_resolution=math.radians(1)):

        x, y, theta = particle_pose  # Unpack particle pose
        num_beams = int(fov / angular_resolution)
        simulated_ranges = []

        for i in range(num_beams):
            # Compute the angle of the current beam
            angle = theta - (fov / 2) + i * angular_resolution
            sin_theta = math.sin(angle)
            cos_theta = math.cos(angle)

            # Step along the ray
            for r in np.arange(0, max_range, self.resolution):
                # Compute the ray's current position in the world
                ray_x = x + r * cos_theta
                ray_y = y + r * sin_theta

                # Convert to grid indices
                cell_x, cell_y = self.world_to_grid(ray_x, ray_y)

                # Check if the ray is out of bounds
                if cell_x < 0 or cell_x >= self.map_width or cell_y < 0 or cell_y >= self.map_height:
                    simulated_ranges.append(max_range)
                    break

                # Check for obstacles
                if self.occupancy_grid[cell_y, cell_x] > 0:  # Assuming >0 means obstacle
                    simulated_ranges.append(r)
                    break
            else:
                # If no obstacle is hit, append max_range
                simulated_ranges.append(max_range)

        return simulated_ranges

    def simulate_lidar_for_particles(self):
        """
        Simulates LiDAR scans for all particles using raycasting.
        """
        if self.occupancy_grid is None or len(self.particles) == 0:
            rospy.logwarn("Map or particles not available for LiDAR simulation.")
            return

        for particle in self.particles:
            simulated_scan = self.raycast(particle)
            self.publish_laser_scan(simulated_scan, particle)
            rospy.loginfo(f"Simulated scan for particle at ({particle[0]:.2f}, {particle[1]:.2f}, {particle[2]:.2f}).")

    def Lidar_callback(self, msg):
        rospy.loginfo("Received LiDAR scan.")


    def publish_laser_scan(self, ranges, particle_pose):
        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = "map"  # Replace with appropriate frame
        scan_msg.angle_min = -math.pi  # Full 360° scan
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = math.radians(1)  # Angular resolution
        scan_msg.range_min = 0.0
        scan_msg.range_max = 10.0  # Example max range
        scan_msg.ranges = ranges

        self.scan_pub.publish(scan_msg)
        rospy.loginfo(f"Published simulated scan for particle at ({particle_pose[0]:.2f}, {particle_pose[1]:.2f}, {particle_pose[2]:.2f}).")


def main():
    rospy.init_node('raycasting', anonymous=True)
    node = LidarData()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
