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

    best_route = find_best_route_v3(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize the starting city
    start_city = 0

    # Create an empty list to store the route
    route = [start_city]

    # Create a list of available cities to visit
    available_cities = list(range(1, len(_distances)))

    # Use the nearest neighbor heuristic to find the next city to visit
    while available_cities:
        current_city = route[-1]
        nearest_city = available_cities[np.argmin(_distances[current_city][available_cities])]
        route.append(nearest_city)
        available_cities.remove(nearest_city)

    # Add the starting city back to the end of the route
    route.append(start_city)

    return tuple(route)
