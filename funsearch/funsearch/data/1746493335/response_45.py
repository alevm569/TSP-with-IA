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


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with hybrid heuristic."""

    # Create a set to track visited cities
    visited = set()

    # Start from a random city
    current_city = np.random.randint(len(_distances))
    visited.add(current_city)

    # Build the route iteratively
    route = [current_city]

    while len(visited) < len(_distances):
        # Find the closest unvisited city
        next_city = np.argmin([_distances[current_city][j] for j in range(len(_distances)) if j not in visited])
        visited.add(next_city)
        route.append(next_city)

        # Update the current city
        current_city = next_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
