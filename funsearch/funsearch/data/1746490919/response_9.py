import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize a random permutation of cities
    permutation = np.random.permutation(len(_distances))

    # Perform local search using a 2-opt neighborhood operator
    while True:
        best_distance = calculate_route_distance(permutation, _distances)

        # Generate a random pair of cities to swap
        i, j = np.random.randint(0, len(_distances), 2)
        permutation[i], permutation[j] = permutation[j], permutation[i]

        # Calculate the distance of the new permutation
        new_distance = calculate_route_distance(permutation, _distances)

        # If the new distance is better, keep it
        if new_distance < best_distance:
            best_distance = new_distance
        else:
            # Otherwise, reverse the change to restore the previous permutation
            permutation[i], permutation[j] = permutation[j], permutation[i]

        # If the best distance does not improve for a certain number of iterations, stop the search
        if best_distance == calculate_route_distance(permutation, _distances):
            break

    return tuple(permutation)
