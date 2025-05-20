import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with hybrid heuristics."""

    # Implement hybrid heuristic using a combination of local search and 2-opt operations.
    # Use funsearch library for efficient implementation of these heuristics.

    # Define the search space and objective function.
    search_space = funsearch.PermutationSearchSpace(len(_distances))
    objective_function = lambda route: calculate_route_distance(route, _distances)

    # Create a hybrid heuristic object.
    hybrid_heuristic = funsearch.HybridHeuristic(
        funsearch.LocalSearch(funsearch.RandomInitialization()),
        funsearch.TwoOptHeuristic()
    )

    # Run the search algorithm.
    best_route = funsearch.greedy_search(
        search_space,
        objective_function,
        hybrid_heuristic,
        max_iterations=1000
    )

    return best_route
