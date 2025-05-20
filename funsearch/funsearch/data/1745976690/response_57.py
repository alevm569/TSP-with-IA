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

    best_route = find_best_route_vx(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`.

    Uses a hybrid heuristic that combines local search with a 2-opt optimization technique.
    """

    # Generate an initial route using the nearest neighbor heuristic
    initial_route = nearest_neighbor(_distances)

    # Perform local search to refine the route
    best_route = local_search(initial_route, _distances)

    # Use 2-opt to further optimize the route
    best_route = two_opt(best_route, _distances)

    return best_route


# Helper functions from find_best_route_v2

def nearest_neighbor(_distances):
    # ...

def local_search(route, _distances):
    # ...

def two_opt(route, _distances):
    # ...
