#!/usr/bin/env python3

import copy

from geometry_msgs.msg import Point

from .graph import is_occluded

class PathSmoother():
    def __init__(self, parent_node, graph, alpha, beta):
        self.parent_node_ = parent_node
        self.graph_ = graph
        
        self.alpha_ = alpha
        self.beta_ = beta

        self.path_smooth_ = []

    def smooth_path(self, path_nodes):
        """Smooth the path to remove sharp corners resulting from the grid-based planning"""

        self.parent_node_.get_logger().info('Smoothing path...')

        # Convert into into a geometry_msgs.Point[]
        path = []

        for node in path_nodes:
            p = Point()
            p.x = float(node.x)
            p.y = float(node.y)
            path.append(p)

        # Initialise the smooth path
        path_smooth = copy.deepcopy(path)

        # Loop until the smoothing converges
        # In each iteration, update every waypoint except the first and last waypoint

        ####################
        ## YOUR CODE HERE ##
        ## Task 5         ##
        ####################

        alpha = self.alpha_
        beta = self.beta_

        epsilon = 0.001  # convergence threshold
        change = epsilon + 1  # initial change set above epsilon to ensure loop starts

        while change > epsilon:
            change = 0.0
            # Loop through all points except first and last
            for i in range(1, len(path) - 1):
                old_s_i_x = path_smooth[i].x
                old_s_i_y = path_smooth[i].y

                # Update s_i according to the smoothing equation
                s_i_x_new = path_smooth[i].x - (alpha + 2 * beta) * path_smooth[i].x + alpha * path[i].x + beta * path_smooth[i - 1].x + beta * path_smooth[i + 1].x
                s_i_y_new = path_smooth[i].y - (alpha + 2 * beta) * path_smooth[i].y + alpha * path[i].y + beta * path_smooth[i - 1].y + beta * path_smooth[i + 1].y

            # Check if the new point is in collision
            # if not self.graph_.is_occluded([old_s_i_x, old_s_i_y], [s_i_x_new, s_i_y_new]):
                # Update smooth path if no obstacle
                path_smooth[i].x = s_i_x_new
                path_smooth[i].y = s_i_y_new

                # Calculate change
                change += (s_i_x_new - old_s_i_x) ** 2 + (s_i_y_new - old_s_i_y) ** 2


        ####################

        self.path_smooth_ = path_smooth