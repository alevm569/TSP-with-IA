import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a hybrid heuristic that combines the nearest neighbor and k-opt techniques.
    # Implement a k-opt neighborhood operator to explore neighboring solutions.

    # Initialize the heuristic
    heuristic = funsearch.KNN(k=2)

    # Run the heuristic to find the best route
    best_route = heuristic.find_best_route(_distances)

    # Return the best route
    return best_route
