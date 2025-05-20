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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2.

    Uses a hybrid heuristic combining nearest neighbor and 2-opt.
    """

    # Initialize a random starting city.
    current_city = np.random.randint(len(matrix_distances))

    # Create an empty route.
    route = [current_city]

    # Build the route using nearest neighbor.
    unvisited_cities = set(range(len(matrix_distances)))
    unvisited_cities.remove(current_city)

    while unvisited_cities:
        # Find the nearest unvisited city.
        nearest_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])
        unvisited_cities.remove(nearest_city)
        route.append(nearest_city)
        current_city = nearest_city

    # Perform 2-opt swaps to improve the route.
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_before = matrix_distances[route[i]][route[j]]
            distance_after = matrix_distances[route[i]][route[j - 1]] + matrix_distances[route[i + 1]][route[j]] - distance_before
            if distance_after < distance_before:
                route[i+1:j] = route[j-1:i:-1]

    return tuple(route)
