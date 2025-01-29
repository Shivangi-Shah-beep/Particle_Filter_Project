#!/usr/bin/env python3

import rospy
from nav_msgs.srv import GetMap
import numpy as np
from sklearn.neighbors import NearestNeighbors

class LikelihoodField:
    def __init__(self):
        # Wait for the map service and fetch the map
        rospy.wait_for_service("static_map")
        static_map_service = rospy.ServiceProxy("static_map", GetMap)
        self.map = static_map_service().map

        # Extract map dimensions and resolution
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution

        # Generate grid cell coordinates
        all_cells = np.zeros((width * height, 2))
        occupied_cells = []

        idx = 0
        for x in range(width):
            for y in range(height):
                # Compute linear index for the occupancy grid
                grid_index = x + y * width
                all_cells[idx] = [float(x), float(y)]
                # Check if the cell is occupied
                if self.map.data[grid_index] > 0:
                    occupied_cells.append([float(x), float(y)])
                idx += 1

        # Convert occupied cells to a NumPy array
        occupied_cells = np.array(occupied_cells)

        # Use NearestNeighbors to compute distances from all cells to the nearest obstacle
        nearest_neighbors = NearestNeighbors(n_neighbors=1, algorithm="ball_tree").fit(occupied_cells)
        distances, _ = nearest_neighbors.kneighbors(all_cells)

        # Populate the closest obstacle distance array
        self.closest_occ = np.zeros((width, height))
        idx = 0
        for x in range(width):
            for y in range(height):
                self.closest_occ[x, y] = distances[idx][0] * resolution
                idx += 1

        # Store occupied cell coordinates for additional computations
        self.occupied_cells = occupied_cells

    def get_bounding_box(self):
        lower_bounds = self.occupied_cells.min(axis=0)
        upper_bounds = self.occupied_cells.max(axis=0)
        res = self.map.info.resolution
        origin = self.map.info.origin.position
        return (
            (lower_bounds[0] * res + origin.x, upper_bounds[0] * res + origin.x),
            (lower_bounds[1] * res + origin.y, upper_bounds[1] * res + origin.y)
        )

    def get_closest_obstacle_distance(self, x, y):
        res = self.map.info.resolution
        origin = self.map.info.origin.position

        # Convert world coordinates to grid indices
        grid_x = (x - origin.x) / res
        grid_y = (y - origin.y) / res

        # Handle single coordinate or arrays
        if isinstance(x, np.ndarray):
            grid_x = grid_x.astype(int)
            grid_y = grid_y.astype(int)
        else:
            grid_x = int(grid_x)
            grid_y = int(grid_y)

        # Check bounds
        if isinstance(x, np.ndarray):
            valid_mask = (grid_x >= 0) & (grid_y >= 0) & (grid_x < self.map.info.width) & (grid_y < self.map.info.height)
            distances = np.full_like(grid_x, float("nan"), dtype=float)
            distances[valid_mask] = self.closest_occ[grid_x[valid_mask], grid_y[valid_mask]]
            return distances
        else:
            if 0 <= grid_x < self.map.info.width and 0 <= grid_y < self.map.info.height:
                return self.closest_occ[grid_x, grid_y]
            else:
                return float("nan")
