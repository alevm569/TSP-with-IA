import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a hybrid approach that combines nearest neighbor and 2-opt heuristics
    best_route = funsearch.hybrid_tsp(
        _distances,
        funsearch.nearest_neighbor,
        funsearch.two_opt,
        max_iterations=1000,
        population_size=100,
        crossover_probability=0.8,
        mutation_probability=0.2,
    )

    return best_route
