import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a hybrid approach.

    Combines the following heuristics:
        - Nearest neighbor
        - Cheapest insertion
        - 2-opt local search

    Returns:
        A tuple representing the best route.
    """

    # Use nearest neighbor to generate an initial tour
    initial_tour = funsearch.nearest_neighbor(_distances)

    # Use cheapest insertion to improve the tour
    improved_tour = funsearch.cheapest_insertion(_distances, initial_tour)

    # Use 2-opt local search to refine the tour
    final_tour = funsearch.two_opt(_distances, improved_tour)

    return final_tour
