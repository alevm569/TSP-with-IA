```python
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

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using hybrid heuristics."""

    # Nearest neighbor heuristic
    initial_route = nearest_neighbor(_distances)

    # Local search with 2-opt neighborhood operator
    best_route = local_search(initial_route, _distances, neighborhood_op=two_opt)

    return best_route

# Helper functions from find_best_route_v2
def nearest_neighbor(_distances: np.ndarray) -> tuple[int, ...]:
    # ...

def local_search(route: tuple[int, ...], distances: np.ndarray, neighborhood_op) -> tuple[int, ...]:
    # ...

def two_opt(route: tuple[int, ...], distances: np.ndarray) -> tuple[int, ...]:
    # ...
