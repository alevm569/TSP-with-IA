import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Generate a random initial solution
    num_cities = len(_distances)
    initial_route = np.random.permutation(num_cities)

    # Perform local search
    best_route = initial_route
    best_distance = calculate_route_distance(best_route, _distances)

    for i in range(100):
        # Generate a new candidate solution by swapping two random cities
        candidate_route = best_route.copy()
        idx1, idx2 = np.random.randint(num_cities, size=2)
        candidate_route[idx1], candidate_route[idx2] = candidate_route[idx2], candidate_route[idx1]

        # Calculate the distance of the candidate route
        candidate_distance = calculate_route_distance(candidate_route, _distances)

        # Update the best route if the candidate route is better
        if candidate_distance < best_distance:
            best_route = candidate_route
            best_distance = candidate_distance

    return best_route
