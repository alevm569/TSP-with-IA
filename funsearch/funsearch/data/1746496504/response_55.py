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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of the find_best_route function using a hybrid approach.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    This function implements a hybrid approach that combines two or more of the following heuristics:

    - Nearest neighbor
    - Cheapest insertion
    - Local search
    - 2-opt
    - ACO (ant colony optimization)

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a random route
    num_cities = len(matrix_distances)
    route = np.random.permutation(num_cities)

    # Use ACO to find an initial solution
    aco = funsearch.ACO(num_cities, matrix_distances)
    aco.run()
    best_route = aco.best_route

    # Perform local search to refine the solution
    local_search = funsearch.LocalSearch(matrix_distances)
    best_route = local_search.run(best_route)

    return best_route
