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

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0` using a hybrid heuristic."""

    # Initialize a random permutation of cities
    cities = np.random.permutation(np.arange(_distances.shape[0]))

    # Create a neighborhood operator to swap two cities in the route
    def swap_operator(route):
        i, j = np.random.randint(len(route), size=2)
        route[i], route[j] = route[j], route[i]
        return route

    # Perform local search using the neighborhood operator
    best_route = funsearch.local_search(cities, swap_operator)

    return best_route
