import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO."""

    # Initialize ACO algorithm
    num_cities = len(_distances)
    aco = funsearch.ACO(num_cities)

    # Run ACO algorithm for a fixed number of iterations
    num_iterations = 1000
    for _ in range(num_iterations):
        aco.run()

    # Get the best route found by ACO
    best_route = aco.best_route()

    return best_route
