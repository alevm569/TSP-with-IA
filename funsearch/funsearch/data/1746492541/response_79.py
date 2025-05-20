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

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`.

    Uses a hybrid approach combining the nearest neighbor heuristic and the 2-opt local search algorithm.

    """

    # Generate an initial route using the nearest neighbor heuristic.
    start_city = 0
    route = [start_city]
    unvisited = list(range(1, len(_distances)))

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Perform local search using the 2-opt algorithm to improve the route.
    random.seed(42)  # Set a seed for reproducibility
    for _ in range(100):  # Number of iterations
        i, j = random.sample(range(len(route)), 2)
        route = funsearch.operators.two_opt(route, i, j)

    return tuple(route)
