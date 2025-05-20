import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with ACO heuristic."""

    # Create an ACO object
    aco = funsearch.ACO(distance_matrix=_distances)

    # Run the ACO algorithm for 100 iterations
    best_route = aco.run(iterations=100)

    # Return the best route as a tuple
    return tuple(best_route)

def find_best_route_v4(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v3` with hybrid heuristic."""

    # Create a hybrid object with ACO and 2-opt heuristics
    hybrid = funsearch.Hybrid(distance_matrix=_distances, heuristics=[funsearch.ACO(), funsearch.TwoOpt()])

    # Run the hybrid algorithm for 100 iterations
    best_route = hybrid.run(iterations=100)

    # Return the best route as a tuple
    return tuple(best_route)
