import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
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
    Improved version of `find_best_route_v2`.

    Uses a hybrid heuristic that combines nearest neighbor and cheapest insertion.
    """

    # Initialize current route and total distance
    current_route = np.random.permutation(len(matrix_distances))
    total_distance = 0

    # Iterate until all cities are visited
    while len(current_route) > 0:
        # Find the nearest unvisited city
        nearest_city = np.argmin([matrix_distances[current_route[-1]][city] for city in current_route if city not in current_route])
        total_distance += matrix_distances[current_route[-1]][nearest_city]

        # Add the nearest city to the route
        current_route = np.append(current_route, nearest_city)

    # Close the route by adding the distance between the first and last cities
    total_distance += matrix_distances[current_route[-1]][current_route[0]]

    return tuple(current_route)
