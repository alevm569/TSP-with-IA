import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO heuristic."""

    # Initialize ACO parameters
    num_ants = 10
    num_iterations = 100
    alpha = 1  # pheromone weight
    beta = 5  # distance weight

    # Create ACO algorithm
    aco = funsearch.algorithms.aco.ACO(
        distances=_distances,
        num_ants=num_ants,
        num_iterations=num_iterations,
        alpha=alpha,
        beta=beta,
    )

    # Run ACO algorithm
    best_route = aco.run()

    return best_route
