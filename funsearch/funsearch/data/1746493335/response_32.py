import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using ACO."""

    # Create an ACO algorithm
    aco = funsearch.aco.ACO(
        distance_matrix=_distances,
        initial_pheromone=1.0,
        evaporation_rate=0.5,
        alpha=1.0,
        beta=2.0,
        num_iterations=100,
        population_size=100
    )

    # Run the ACO algorithm
    best_route = aco.run()

    return best_route
