import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

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

    # Initialize the first city as the starting point
    start_city = 0
    current_city = start_city
    visited = set([current_city])

    # Create a list to store the route
    route = [current_city]

    # Iterate until all cities have been visited
    while len(visited) < len(_distances):

        # Find the closest unvisited city
        min_distance = float('inf')
        for i in range(len(_distances)):
            if i not in visited:
                distance = _distances[current_city][i]
                if distance < min_distance:
                    min_distance = distance
                    next_city = i

        # Update the current city and add it to the route
        current_city = next_city
        visited.add(current_city)
        route.append(current_city)

    # Return to the starting city
    route.append(start_city)

    return tuple(route)
