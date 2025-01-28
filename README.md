# particle_filter_project

## Acknowledgment
This project builds upon the particle filter project explained here: https://classes.cs.uchicago.edu/archive/2022/spring/20600-1/particle_filter_project.html#deadlines.

### Extensions and Modifications
This repository extends the original implementation by:
- Adding a **Likelihood Field Model** to improve measurement accuracy using a probabilistic approach for sensor data.
- Implementing a **dynamic particle initialization method** that uses the map's free space for uniform distribution.
- Incorporating **motion model** with Gaussian noise for realistic particle updates.
- Developing a **measurement model** that evaluates particle weights based on sensor likelihood using a zero-centered Gaussian.

These improvements aim to enhance localization accuracy and robustness in real-world robotic applications.



## Objective

The goal of this project is to implement a custom Particle Filter (PF) for localization, integrating
data from multiple sensors (LiDAR and odometry) in a simulated ROS and Gazebo
environment. 

## The Particle Filter Localization

The goal of our particle filter localization (i.e., Monte Carlo localization) will be to help a robot answer the question of "where am I"? This problem assumes that the robot has a map of its environment, however, the robot either does not know or is unsure of its position and orientation within that environment.

The way that a robot determines its location using a particle filter localization is similar to how people used to find their way around unfamiliar places using physical maps . In order to determine your location on a map, you would look around for landmarks or street signs, and then try to find places on the map that matched what you were seeing in real life.

The particle filter localization makes many guesses (particles) for where it might think the robot could be, all over the map. Then, it compares what it's seeing (using its sensors) with what each guess (each particle) would see. Guesses that see similar things to the robot are more likely to be the true position of the robot. As the robot moves around in its environment, it should become clearer and clearer which guesses are the ones that are most likely to correspond to the actual robot's position.

In more detail, the particle filter localization first initializes a set of particles in random locations and orientations within the map and then iterates over the following steps until the particles have converged to (hopefully) the position of the robot:

1. Capture the movement of the robot from the robot's odometry
2. Update the position and orientation of each of the particles based on the robot's movement
3. Compare the laser scan of the robot with the hypothetical laser scan of each particle, assigning each particle a weight that corresponds to how similar the particle's hypothetical laser scan is to the robot's laser scan
4. Resample with replacement a new set of particles probabilistically according to the particle weights
5. Update our estimate of the robot's location

## Using SLAM to Create a Map of Your Environment
Our first step with this project involves recording a map of the maze . We records the map using the built in turtlebot3 SLAM tools. We first run  Gazebo simulator. For this project, we're using a simulated version of the particle filter maze, then we run the SLAM node. Teleoperating the Turtlebot3 around the environment until we get a complete map and  save our map.

## Running the Code
First terminal: run roscore
$ roscore

Second terminal: run our Gazebo simulator. For this project, we're using a simulated version of the particle filter maze.
 $ roslaunch particle_filter_project turtlebot3_maze.launch

Third terminal: launch the launchfile that we've constructed that 1. starts up the map server 2. sets up some important coordinate transformations, and 3. runs rviz with some helpful configurations (visualizing the map, particle cloud, robot location).
 $ roslaunch particle_filter_project visualize_particles.launch

Fourth terminal: run the Turtlebot3 provided code to teleoperate the robot.
 $ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Fifth terminal: run the particle filter code.
 $ rosrun particle_filter_project particle_filter.py






