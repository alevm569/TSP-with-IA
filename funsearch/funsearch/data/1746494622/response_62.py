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

    Uses a hybrid approach combining nearest neighbor and 2-opt heuristics.
    """

    # Nearest neighbor heuristic to generate an initial solution
    start_city = 0
    route = [start_city]
    unvisited = set(range(1, len(matrix_distances)))

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: matrix_distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # 2-opt heuristic to improve the solution
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_original = matrix_distances[route[i]][route[j]]

            # Swap the two cities
            route[i], route[j] = route[j], route[i]

            distance_improved = matrix_distances[route[i]][route[j]]

            if distance_improved < distance_original:
                break

    return tuple(route)
