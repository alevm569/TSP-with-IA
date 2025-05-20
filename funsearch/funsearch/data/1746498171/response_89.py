import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a hybrid heuristic combining local search and ACO
    # Initialize ACO with a suitable number of ants and iterations
    aco = funsearch.ACO(distance_matrix=_distances)

    # Run ACO to find an initial solution
    best_route_aco = aco.run()

    # Perform local search on the ACO solution
    local_search = funsearch.LocalSearch(distance_matrix=_distances)
    best_route_ls = local_search.run(best_route_aco)

    # Return the best route found through the hybrid heuristic
    return best_route_ls
