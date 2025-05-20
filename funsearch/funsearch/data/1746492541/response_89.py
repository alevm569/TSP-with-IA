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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2.

    Uses a combination of local search and 2-opt heuristics.
    """

    # Generate an initial route using nearest neighbor
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(len(distances)))

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Local search
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                # Perform a 2-opt swap
                new_route = route[:i] + route[j:i:-1] + route[j+1:]
                new_distance = calculate_route_distance(new_route, distances)

                if new_distance < calculate_route_distance(route, distances):
                    route = new_route

    return tuple(route)
