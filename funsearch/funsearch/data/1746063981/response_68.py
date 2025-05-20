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


def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Perform a local search using the 2-opt heuristic.
    current_route = np.arange(len(_distances))
    best_route = current_route
    best_distance = calculate_route_distance(best_route, _distances)

    while True:
        for i in range(len(current_route)):
            for j in range(i + 1, len(current_route)):
                new_route = current_route.copy()
                new_route[i:j+1] = new_route[j:i:-1]
                new_distance = calculate_route_distance(new_route, _distances)

                if new_distance < best_distance:
                    best_distance = new_distance
                    best_route = new_route

        if best_distance == calculate_route_distance(current_route, _distances):
            break
        else:
            current_route = best_route

    return best_route
