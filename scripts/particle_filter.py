#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
import math

from random import uniform, random

from likelihood_field import LikelihoodField


def get_yaw_from_pose(p):
    return euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])[2]

def euler_from_quaternion(quat):
    
    x, y, z, w = quat
    siny_cosp = 2.0*(w*z + x*y)
    cosy_cosp = 1.0 - 2.0*(y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (0.0, 0.0, yaw)

def compute_prob_zero_centered_gaussian(dist, sd):
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-dist**2)/(2 * sd**2))
    return prob

class Particle:
    def __init__(self, pose, w):
        # particle pose (Pose object from geometry_msgs)
        self.pose = pose
        # particle weight
        self.w = w

class ParticleFilter:
    def __init__(self):
        
        self.initialized = False        

        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter', anonymous=True)

        
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 10000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we perform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None

        # Setup publishers and subscribers
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # Wait for map before initializing particles
        rospy.wait_for_message(self.map_topic, OccupancyGrid)

        # Initialize likelihood field
        self.likelihood_field = LikelihoodField()

        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True


    def get_map(self, data):
        self.map = data
    

    def initialize_particle_cloud(self):
        # Initialize particles uniformly in free space
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        data = np.array(self.map.data).reshape((height, width))

        free_cells = np.argwhere(data == 0)  # free cells
        if len(free_cells) == 0:
            # No free space?? Just place all particles at origin
            for _ in range(self.num_particles):
                p = Pose()
                p.position.x = origin_x
                p.position.y = origin_y
                yaw = uniform(-math.pi, math.pi)
                q = quaternion_from_euler(0,0,yaw)
                p.orientation.x = q[0]
                p.orientation.y = q[1]
                p.orientation.z = q[2]
                p.orientation.w = q[3]
                self.particle_cloud.append(Particle(p, 1.0/self.num_particles))
        else:
            indices = np.random.choice(len(free_cells), self.num_particles, replace=True)
            for idx in indices:
                cell_y, cell_x = free_cells[idx]
                px = origin_x + cell_x * resolution
                py = origin_y + cell_y * resolution
                yaw = uniform(-math.pi, math.pi)

                p = Pose()
                p.position.x = px
                p.position.y = py
                q = quaternion_from_euler(0,0,yaw)
                p.orientation.x = q[0]
                p.orientation.y = q[1]
                p.orientation.z = q[2]
                p.orientation.w = q[3]

                self.particle_cloud.append(Particle(p, 1.0/self.num_particles))

        self.normalize_particles()
        self.publish_particle_cloud()


    def normalize_particles(self):
        total_weight = sum(p.w for p in self.particle_cloud)
        if total_weight > 0:
            for p in self.particle_cloud:
                p.w /= total_weight


    def publish_particle_cloud(self):
        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)


    def publish_estimated_robot_pose(self):
        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)


    def resample_particles(self):
        weights = np.array([p.w for p in self.particle_cloud])
        weights /= np.sum(weights)
        indices = np.random.choice(len(self.particle_cloud), size=self.num_particles, p=weights)
        new_particles = []
        for i in indices:
            old_p = self.particle_cloud[i]
            p = Particle(Pose(), 1.0/self.num_particles)
            p.pose.position.x = old_p.pose.position.x
            p.pose.position.y = old_p.pose.position.y
            p.pose.position.z = old_p.pose.position.z
            p.pose.orientation.x = old_p.pose.orientation.x
            p.pose.orientation.y = old_p.pose.orientation.y
            p.pose.orientation.z = old_p.pose.orientation.z
            p.pose.orientation.w = old_p.pose.orientation.w
            new_particles.append(p)
        self.particle_cloud = new_particles


    def robot_scan_received(self, data):
        if not self.initialized:
            return

        # We try to get transforms at rospy.Time(0) for the latest available transform
        if not self.tf_listener.canTransform(self.base_frame, data.header.frame_id, rospy.Time(0)):
            return

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, 
            PoseStamped(header=Header(stamp=rospy.Time(0), frame_id=data.header.frame_id)))

        if not self.tf_listener.canTransform(self.odom_frame, self.base_frame, rospy.Time(0)):
            return

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, 
            PoseStamped(header=Header(stamp=rospy.Time(0), frame_id=self.base_frame), pose=Pose()))

        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return

        if self.particle_cloud:
            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            # check if we've moved enough
            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                self.update_particles_with_motion_model()
                self.update_particle_weights_with_measurement_model(data)
                self.normalize_particles()
                self.resample_particles()
                self.update_estimated_robot_pose()
                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose


    def update_estimated_robot_pose(self):
        x_sum = 0.0
        y_sum = 0.0
        sin_sum = 0.0
        cos_sum = 0.0
        total_w = 0.0
        for p in self.particle_cloud:
            w = p.w
            x_sum += p.pose.position.x * w
            y_sum += p.pose.position.y * w
            yaw = get_yaw_from_pose(p.pose)
            sin_sum += math.sin(yaw)*w
            cos_sum += math.cos(yaw)*w
            total_w += w

        if total_w > 0:
            mean_x = x_sum / total_w
            mean_y = y_sum / total_w
            mean_yaw = math.atan2(sin_sum, cos_sum)

            self.robot_estimate.position.x = mean_x
            self.robot_estimate.position.y = mean_y
            q = quaternion_from_euler(0, 0, mean_yaw)
            self.robot_estimate.orientation.x = q[0]
            self.robot_estimate.orientation.y = q[1]
            self.robot_estimate.orientation.z = q[2]
            self.robot_estimate.orientation.w = q[3]


    def update_particle_weights_with_measurement_model(self, data):
        # Measurement model:
        # We'll consider a small subset of beams. For simplicity, let's do something similar
        # Use a gaussian sensor model centered at 0 with some std dev:
        sd = 0.2

        # We'll pick just the front beam (index 0) for demonstration:
        beam_idx = 0
        obs_range = data.ranges[beam_idx]

        # The angle of beam_idx:
        beam_angle = data.angle_min + beam_idx * data.angle_increment

        for p in self.particle_cloud:
            px = p.pose.position.x
            py = p.pose.position.y
            pyaw = get_yaw_from_pose(p.pose)

            # Compute beam endpoint in map frame
            end_x = px + obs_range * math.cos(pyaw + beam_angle)
            end_y = py + obs_range * math.sin(pyaw + beam_angle)

            # Get distance to closest obstacle from likelihood field
            dist = self.likelihood_field.get_closest_obstacle_distance(end_x, end_y)
            if math.isnan(dist):
                # If out of map, give a very low probability
                prob = 1e-6
            else:
                # Compute probability from the Gaussian
                prob = compute_prob_zero_centered_gaussian(dist, sd)

            # Update particle weight
            p.w *= prob


    def update_particles_with_motion_model(self):
        # Use odometry difference to update particle positions
        old_x = self.odom_pose_last_motion_update.pose.position.x
        old_y = self.odom_pose_last_motion_update.pose.position.y
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        curr_x = self.odom_pose.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)

        dx = curr_x - old_x
        dy = curr_y - old_y
        dyaw = curr_yaw - old_yaw

        trans = math.sqrt(dx*dx + dy*dy)
        rot = dyaw

        # noise parameters
        trans_noise = 0.05
        rot_noise = 0.05

        for p in self.particle_cloud:
            yaw = get_yaw_from_pose(p.pose)
            noisy_trans = np.random.normal(trans, trans_noise)
            noisy_rot = np.random.normal(rot, rot_noise)

            new_x = p.pose.position.x + noisy_trans * math.cos(yaw)
            new_y = p.pose.position.y + noisy_trans * math.sin(yaw)
            new_yaw = yaw + noisy_rot

            q = quaternion_from_euler(0, 0, new_yaw)
            p.pose.position.x = new_x
            p.pose.position.y = new_y
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]


if __name__=="__main__":
    pf = ParticleFilter()
    rospy.spin()













