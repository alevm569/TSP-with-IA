import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using hybrid heuristics."""

    # Hybrid heuristic: Combine nearest neighbor and cheapest insertion
    def hybrid_heuristic(distances):
        # Initial route using nearest neighbor
        route = funsearch.nearest_neighbor(distances)

        # Randomly insert remaining cities using cheapest insertion
        remaining_cities = set(range(len(distances))) - set(route)
        while remaining_cities:
            city = np.random.choice(list(remaining_cities))
            best_insertion_index = funsearch.cheapest_insertion(route, city, distances)
            route.insert(best_insertion_index, city)
            remaining_cities.remove(city)

        return route

    # Use hybrid heuristic to generate initial route
    initial_route = hybrid_heuristic(_distances)

    # Local search optimization
    best_route = funsearch.local_search(initial_route, _distances)

    return best_route
