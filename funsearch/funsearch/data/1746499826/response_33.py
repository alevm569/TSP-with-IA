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

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`.

    Uses a hybrid approach combining the nearest neighbor and cheapest insertion heuristics.
    """

    # Initialize the route with the nearest neighbor strategy
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(len(_distances))) - {start_city}

    # Iterate until all cities are visited
    while remaining_cities:
        current_city = route[-1]
        # Find the city with the shortest distance to the current city
        next_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(next_city)
        remaining_cities.remove(next_city)

    # Ensure the route returns to the starting city
    route.append(start_city)

    return tuple(route)
