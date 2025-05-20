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

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid approach.

    Hybrid approach combining:
        - Local search with a 2-opt heuristic for route improvement.
        - ACO algorithm for initial route generation.

    """
    num_cities = len(distances)

    # Generate an initial route using ACO
    aco = funsearch.ACO(num_cities)
    initial_route = aco.find_best_route(distances)

    # Perform local search with 2-opt heuristic
    local_search = funsearch.LocalSearch(distances)
    best_route = local_search.improve_route(initial_route)

    return best_route
