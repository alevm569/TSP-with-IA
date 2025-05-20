import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO."""
    num_cities = len(_distances)
    aco = funsearch.ACO(num_cities, _distances)

    best_route = aco.solve()
    return best_route

def find_best_route_v4(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v3` using Genetic Algorithms."""
    num_cities = len(_distances)
    ga = funsearch.GA(num_cities, _distances)

    best_route = ga.solve()
    return best_route
