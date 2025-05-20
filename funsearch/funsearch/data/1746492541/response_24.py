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

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using hybrid heuristics."""

    # Nearest neighbor heuristic
    current_city = 0
    route = [current_city]
    unvisited_cities = set(range(len(_distances))) - {current_city}

    while unvisited_cities:
        nearest_city = min(unvisited_cities, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)
        current_city = nearest_city

    # Local search heuristic
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:]
            new_route[i], new_route[j] = new_route[j], new_route[i]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return tuple(route)
