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

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Use a hybrid heuristic that combines the nearest neighbor and cheapest insertion heuristics
    # Initialize the route with the nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    remaining_cities = list(range(1, len(_distances)))

    # Iterate until all cities have been visited
    while remaining_cities:
        # Get the last city in the route
        current_city = route[-1]

        # Find the cheapest city to add to the route
        cheapest_city = remaining_cities[0]
        min_distance = _distances[current_city][cheapest_city]

        for city in remaining_cities[1:]:
            distance = _distances[current_city][city]
            if distance < min_distance:
                cheapest_city = city
                min_distance = distance

        # Add the cheapest city to the route
        route.append(cheapest_city)
        remaining_cities.remove(cheapest_city)

    # Add the starting city back to the route
    route.append(start_city)

    return tuple(route)
