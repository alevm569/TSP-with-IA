import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route_v1 function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`.

    - Uses a combination of the nearest neighbor and cheapest insertion heuristics.
    - Implements a local search strategy to improve the initial solution.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route using the nearest neighbor heuristic.
    start_city = 0
    route = [start_city]
    unvisited_cities = set(range(len(_distances))) - {start_city}

    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Use the cheapest insertion heuristic to further improve the route.
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            if _distances[route[i]][route[j]] > _distances[route[i]][route[(j + 1) % len(route)]] + _distances[route[(j + 1) % len(route)]][route[j]]:
                route[i], route[j] = route[j], route[i]

    # Perform local search to further improve the route.
    for _ in range(10):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                if _distances[route[i]][route[j]] < _distances[route[i]][route[(j - 1) % len(route)]] + _distances[route[(j - 1) % len(route)]][route[j]]:
                    route[i], route[j] = route[j], route[i]

    return tuple(route)
