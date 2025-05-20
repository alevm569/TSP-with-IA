import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2`.

    Uses a hybrid approach combining:
    - nearest neighbor heuristic for initial route construction
    - 2-opt heuristic for route optimization

    Returns a permutation of cities that minimizes the total route distance.
    """

    # Initial route using nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    unvisited_cities = set(range(len(_distances)))
    unvisited_cities.remove(start_city)

    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Optimize route using 2-opt heuristic
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            if calculate_route_distance(route, _distances) > calculate_route_distance(route[:i] + route[j:i:-1] + route[j + 1:], _distances):
                route = route[:i] + route[j:i:-1] + route[j + 1:]

    return tuple(route)
