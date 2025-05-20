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
    # Use a hybrid approach combining nearest neighbor and 2-opt heuristics
    def hybrid_heuristic(distances):
        # Nearest neighbor heuristic
        start = 0
        route = [start]
        unvisited = set(range(1, len(distances)))

        while unvisited:
            current = route[-1]
            nearest = min(unvisited, key=lambda city: distances[current][city])
            route.append(nearest)
            unvisited.remove(nearest)

        # 2-opt heuristic to improve route
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                distance_difference = distances[route[i]][route[j]] + distances[route[i + 1]][route[j - 1]] - distances[route[i]][route[i + 1]] - distances[route[j]][route[j - 1]]
                if distance_difference < 0:
                    route[i + 1:j] = reversed(route[i + 1:j])

        return tuple(route)

    # Use the hybrid heuristic to find the best route
    best_route = hybrid_heuristic(_distances)

    return best_route
