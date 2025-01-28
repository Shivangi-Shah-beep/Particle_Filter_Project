import rospy
from nav_msgs.msg import OccupancyGrid
from sklearn.neighbors import KDTree
import numpy as np


class LikelihoodField:
    def __init__(self):
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
        self.occupied_cells = []  # Stores coordinates of occupied cells
        self.kd_tree = None  # KD-Tree for occupied cells
        self.resolution = None  # Map resolution
        self.origin_x = None  # Map origin X
        self.origin_y = None  # Map origin Y
        self.map_width = None  # Map width in grid cells
        self.map_height = None  # Map height in grid cells

    def map_callback(self, msg):
        # Extract map dimensions and data
        self.map_height = msg.info.height
        self.map_width = msg.info.width
        data = msg.data
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        # Identify and store occupied cells
        self.occupied_cells = self.occupied(data, self.map_width)

        if len(self.occupied_cells) == 0:
            rospy.logwarn("No occupied cells found. Likelihood field may not work as expected.")
            return

        # Build the KD-Tree
        self.build_kd_tree()

    def occupied(self, data, width):
        occupied = []
        for index in range(len(data)):
            if data[index] > 0:  # Cell is occupied
                x = index % width
                y = index // width
                occupied.append((x, y))
        return occupied

    def build_kd_tree(self):
        if len(self.occupied_cells) == 0:
            rospy.logwarn("No occupied cells found. KD-Tree not built.")
            return

        occupied_array = np.array(self.occupied_cells)
        self.kd_tree = KDTree(occupied_array, metric='euclidean')
        rospy.loginfo("KD-Tree built successfully.")

    def query_kd_tree(self, x, y):
        if self.kd_tree is None:
            rospy.logwarn("KD-Tree not initialized. Returning NaN.")
            return float('nan')

        if x < 0 or x >= self.map_width or y < 0 or y >= self.map_height:
            rospy.logwarn(f"Point ({x}, {y}) is out of bounds.")
            return float('nan')

        query_point = np.array([[x, y]])
        distance, _ = self.kd_tree.query(query_point, k=1)
        return distance[0][0] * self.resolution

    def get_closest_obstacle_distance(self, world_x, world_y):
        grid_x, grid_y = self.world_to_grid(world_x, world_y)
        return self.query_kd_tree(grid_x, grid_y)

    def world_to_grid(self, world_x, world_y):
        grid_x = int((world_x - self.origin_x) / self.resolution)
        grid_y = int((world_y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        world_x = grid_x * self.resolution + self.origin_x
        world_y = grid_y * self.resolution + self.origin_y
        return world_x, world_y
