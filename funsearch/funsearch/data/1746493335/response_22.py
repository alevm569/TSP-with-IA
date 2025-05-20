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
    Improved version of `find_best_route_v0`.

    Uses a combination of local search and 2-opt heuristics.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Generate an initial route using the nearest neighbor heuristic
    initial_route = nearest_neighbor(distances)

    # Perform local search to find a better solution
    best_route = local_search(initial_route, distances)

    # Use 2-opt to further improve the route
    best_route = two_opt(best_route, distances)

    return best_route


# Helper functions from find_best_route_v0

def nearest_neighbor(distances: np.ndarray) -> tuple[int, ...]:
    # ...

def local_search(route: tuple[int, ...], distances: np.ndarray) -> tuple[int, ...]:
    # ...

def two_opt(route: tuple[int, ...], distances: np.ndarray) -> tuple[int, ...]:
    # ...

# End of helper functions
