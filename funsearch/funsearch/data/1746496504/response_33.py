import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use ACO (ant colony optimization) with a heuristic that combines the nearest neighbor and cheapest insertion heuristics.
    aco = funsearch.aco.ACS(
        distance_matrix=_distances,
        heuristic_combination='sum',
        initial_tour=funsearch.tour.nearest_neighbor(_distances),
        num_ants=10,
        iterations=1000,
        seed=42
    )

    best_route = aco.solve()

    return best_route.cities
