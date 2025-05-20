```python
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

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    This function combines the nearest neighbor and cheapest insertion heuristics,
    with a local search optimization.
    """

    # Initial route using nearest neighbor heuristic
    route = nearest_neighbor(distances)

    # Optimize the route using cheapest insertion heuristic
    route = cheapest_insertion(route, distances)

    # Perform local search optimization
    route = local_search(route, distances)

    return route

# Heuristics and optimization functions can be implemented here.
