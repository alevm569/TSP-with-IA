import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a combination of nearest neighbor and 2-opt heuristics.

    Args:
        matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Initialize a random starting city
    current_city = np.random.randint(len(matrix_distances))

    # Create an empty list to store the route
    route = []

    # Iterate until all cities have been visited
    while len(route) < len(matrix_distances):
        # Add the current city to the route
        route.append(current_city)

        # Find the nearest unvisited city
        nearest_city = -1
        min_distance = float('inf')
        for i in range(len(matrix_distances)):
            if i not in route:
                distance = matrix_distances[current_city][i]
                if distance < min_distance:
                    nearest_city = i
                    min_distance = distance

        # Move to the nearest city
        current_city = nearest_city

    # Perform 2-opt optimization to improve the route
    funsearch.optimize.two_opt(route, matrix_distances)

    # Return the route as a tuple
    return tuple(route)
