import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement your new heuristic here.
    # For example, you could use a combination of nearest neighbor and cheapest insertion.

    # Create a set to track visited cities.
    visited = set()

    # Start from the first city.
    current_city = 0

    # Build the route by iteratively selecting the next city.
    route = []
    while len(visited) < len(_distances):
        # Find the nearest unvisited city.
        nearest_city = np.argmin(_distances[current_city][[i for i in range(len(_distances)) if i not in visited]])

        # Add the nearest city to the route and mark it as visited.
        route.append(nearest_city)
        visited.add(nearest_city)

        # Update the current city.
        current_city = nearest_city

    # Close the route by returning to the starting city.
    route.append(route[0])

    return tuple(route)
