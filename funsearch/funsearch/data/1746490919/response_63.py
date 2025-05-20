import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use ACO (ant colony optimization) to generate an initial route.
    num_ants = 10
    num_iterations = 100
    alpha = 1.0  # Weight of pheromone contribution
    beta = 2.0  # Weight of distance cost
    rho = 0.1  # Pheromone evaporation rate

    # Create an ACO solver object.
    aco_solver = funsearch.ACO(_distances, num_ants, num_iterations, alpha, beta, rho)

    # Run the ACO solver to find the best route.
    best_route = aco_solver.solve()

    # Return the best route found.
    return best_route
