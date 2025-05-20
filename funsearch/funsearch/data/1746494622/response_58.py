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
    """Improved version of `find_best_route_v1`."""
    # Use a combination of local search and ACO heuristics
    # Initialize the ACO and local search objects
    aco = funsearch.ACO(_distances)
    local_search = funsearch.LocalSearch(_distances)

    # Run the ACO algorithm to find a candidate route
    candidate_route = aco.solve()

    # Run the local search algorithm to refine the candidate route
    best_route = local_search.solve(candidate_route)

    return best_route
