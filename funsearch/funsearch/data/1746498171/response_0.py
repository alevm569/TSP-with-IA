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

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`.

    Uses a hybrid approach combining nearest neighbor and 2-opt heuristics.
    """

    # Nearest neighbor heuristic
    start_city = np.random.randint(len(_distances))
    current_city = start_city
    route = [current_city]

    while len(route) < len(_distances):
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # 2-opt heuristic
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_before = _distances[route[i]][route[j]]
            distance_after = _distances[route[i]][route[(j - 1) % len(route)]] + _distances[route[j]][route[(i + 1) % len(route)]]

            if distance_after < distance_before:
                route[i+1:j] = route[j-1:i:-1]

    return tuple(route)
