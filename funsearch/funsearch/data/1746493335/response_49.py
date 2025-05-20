import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with hybrid heuristic."""

    # Implement a hybrid heuristic that combines two or more of the following heuristics:
    # - nearest neighbor
    # - cheapest insertion
    # - local search
    # - 2-opt
    # - k-opt

    # Example hybrid heuristic:
    route = np.random.permutation(len(_distances))
    while True:
        best_distance = calculate_route_distance(route, _distances)
        best_move = None

        # Local search
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = route.copy()
                new_route[i], new_route[j] = new_route[j], new_route[i]
                new_distance = calculate_route_distance(new_route, _distances)

                if new_distance < best_distance:
                    best_distance = new_distance
                    best_move = (i, j)

        if best_move is None:
            break
        else:
            route[best_move[0]], route[best_move[1]] = route[best_move[1]], route[best_move[0]]

    return route
