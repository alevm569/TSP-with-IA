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

    # Nearest neighbor heuristic to generate an initial route
    current_city = 0
    route = [current_city]
    visited = set([current_city])

    while len(visited) < len(_distances):
        next_city = np.argmin([_distances[current_city][city] for city in range(len(_distances)) if city not in visited])
        route.append(next_city)
        visited.add(next_city)
        current_city = next_city

    # 2-opt heuristic to improve the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_original = _distances[route[i]][route[j]]
            distance_reversed = _distances[route[i]][route[(j + 1) % len(route)]] + _distances[route[j]][route[(i + 1) % len(route)]] - distance_original

            if distance_reversed < distance_original:
                route[i:j+1] = route[j:i:-1]

    return tuple(route)
