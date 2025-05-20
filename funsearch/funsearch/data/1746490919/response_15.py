import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO."""
    # Initialize ACO parameters
    num_ants = 10
    alpha = 1.0
    beta = 2.0
    rho = 0.1
    Q = 10

    # Create an ACO object
    aco = funsearch.ACO(
        num_ants=num_ants,
        alpha=alpha,
        beta=beta,
        rho=rho,
        Q=Q,
        distance_matrix=_distances,
    )

    # Run ACO for a specified number of iterations
    num_iterations = 1000
    aco.run(num_iterations)

    # Return the best route found by ACO
    return aco.best_route
