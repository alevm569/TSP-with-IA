import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid heuristic."""

    # Initialize a random permutation of cities
    current_route = np.random.permutation(len(_distances))

    # Local search with 2-opt heuristic
    for _ in range(100):
        # Select two random cities in the route
        i, j = np.random.randint(0, len(current_route), size=2)

        # Apply 2-opt swap to the route
        current_route[i:j+1] = current_route[j:i-1:-1]

    # Return the best route found
    return current_route
