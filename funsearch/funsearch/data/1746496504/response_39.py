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
    """Improved version of `find_best_route_v1` using a hybrid heuristic."""

    # Initialize route with a random city
    route = np.random.randint(len(_distances))

    # Iterate until all cities are included
    while len(route) < len(_distances):
        # Find the city that minimizes distance to the last city in the route
        best_city = np.argmin(_distances[route[-1]])

        # Avoid revisiting cities
        if best_city not in route:
            route = np.append(route, best_city)

    # Return to the starting city
    route = np.append(route, route[0])

    return tuple(route)
