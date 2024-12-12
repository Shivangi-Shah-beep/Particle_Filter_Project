#!/usr/bin/env python3
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

class GenerateParticles():
    def __init__(self):
        rospy.Subscriber('/predicted_pose', PoseStamped, self.pose_callback, queue_size=10)

        # Publishers
        self.particles_pub = rospy.Publisher('/particles', PoseArray, queue_size=10)

        self.predicted_pose = None
        self.global_localization = True  # Start with global localization

    def pose_callback(self, msg):
        self.predicted_pose = msg
        if self.predicted_pose is not None:
            rospy.loginfo(f"The predicted pose is: {self.predicted_pose}")
        else:
            rospy.logwarn("Predicted pose not received")

    def generate_particles(self, num_particles=1000, map_bounds=((-10.0, 9.2), (-10.0, 9.2)), std_dev=(0.0072, 0.0063, 0.0056)):
        particles = []

        if self.global_localization:
            rospy.loginfo("Generating particles for global localization...")
            # Uniformly distribute particles within map bounds
            for _ in range(num_particles):
                x_particle = np.random.uniform(map_bounds[0][0], map_bounds[0][1])
                y_particle = np.random.uniform(map_bounds[1][0], map_bounds[1][1])
                theta_particle = np.random.uniform(-np.pi, np.pi)
                particles.append((x_particle, y_particle, theta_particle))
        else:
            rospy.logwarn("Global localization is disabled. No particles generated.")

        return particles

    def publish_particles(self, particles):
        # Create PoseArray message
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        for particle in particles:
            pose = Pose()
            pose.position.x = particle[0]
            pose.position.y = particle[1]

            # Convert theta to quaternion
            quaternion = tf.transformations.quaternion_from_euler(0, 0, particle[2])
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            pose_array.poses.append(pose)

        # Publish particles to /particles topic
        self.particles_pub.publish(pose_array)
        rospy.loginfo("Published particles to /particles topic.")

    def run(self):
        if self.global_localization:
            rospy.loginfo("Running global localization...")
        else:
            rospy.loginfo("Global localization is disabled.")

        particles = self.generate_particles()

        # Publish particles to the /particles topic
        self.publish_particles(particles)


def main():
    rospy.init_node('particle_generator', anonymous=True)
    particle_generator = GenerateParticles()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        particle_generator.run()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
