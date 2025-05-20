import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a hybrid heuristic that combines the nearest neighbor and cheapest insertion heuristics
    def hybrid_heuristic(distances):
        # Start with the nearest neighbor heuristic
        route = np.random.permutation(len(distances))
        current_city = route[0]

        # Perform cheapest insertion for the remaining cities
        for _ in range(1, len(distances)):
            min_distance = np.inf
            next_city = None

            for city in range(len(distances)):
                if city not in route:
                    distance = distances[current_city][city]
                    if distance < min_distance:
                        min_distance = distance
                        next_city = city

            route = np.insert(route, np.where(route == current_city)[0] + 1, next_city)
            current_city = next_city

        return route

    # Use the hybrid heuristic to generate a candidate route
    candidate_route = hybrid_heuristic(_distances)

    # Perform local search to refine the candidate route
    best_route = funsearch.local_search(candidate_route, _distances, funsearch.two_opt)

    return best_route
