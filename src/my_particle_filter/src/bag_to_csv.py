#!/usr/bin/env python3
import rosbag
import csv

def bag_to_csv(bag_file, output_csv, topic):
    bag_file = '/home/shivangi_shah/my_project_ws/src/my_particle_filter/src/2024-12-11-13-45-02.bag'
    output_csv = '/home/shivangi_shah/my_project_ws/src/my_particle_filter/src/pose_error_3.csv'
    topic = '/pose_error'
    with rosbag.Bag(bag_file, 'r') as bag:
        with open(output_csv, 'w') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['time', 'x', 'y', 'theta'])  # Header for CSV

            for topic, msg, t in bag.read_messages(topics=[topic]):
                time = t.to_sec()
                x = msg.x
                y = msg.y
                theta = msg.z  # Assuming angular error is in z
                csv_writer.writerow([time, x, y, theta])

    print(f"Topic {topic} has been written to {output_csv}.")

if __name__ == '__main__':
    # Modify these variables as needed
    bag_file = '/path/to/your.bag'  # Path to your ROS bag file
    output_csv = '/path/to/output.csv'  # Path for the output CSV file
    topic = '/pose_error'  # Topic to extract

    bag_to_csv(bag_file, output_csv, topic)
