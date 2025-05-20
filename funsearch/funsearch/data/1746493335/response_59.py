import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with hybrid heuristic."""

    # Create an ACO optimizer
    aco_optimizer = funsearch.ACOOptimizer(_distances)

    # Create a local search optimizer
    local_search_optimizer = funsearch.LocalSearchOptimizer(_distances)

    # Combine ACO and local search heuristics
    hybrid_optimizer = funsearch.HybridOptimizer(aco_optimizer, local_search_optimizer)

    # Find the best route using the hybrid heuristic
    best_route = hybrid_optimizer.optimize()

    return best_route
