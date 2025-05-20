import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route_v3(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use a hybrid approach combining nearest neighbor and 2-opt heuristics.
    # Initialize a random route.
    np.random.seed(42)
    initial_route = np.random.permutation(len(_distances))

    # Use nearest neighbor to find the initial route.
    def nearest_neighbor(current_city):
        distances = _distances[current_city]
        unvisited_cities = set(range(len(_distances))) - {current_city}
        return min(unvisited_cities, key=lambda c: distances[c])

    # Apply 2-opt to improve the route.
    def two_opt(route):
        best_distance = calculate_route_distance(route, _distances)
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = route[:i] + route[i:j+1][::-1] + route[j+1:]
                new_distance = calculate_route_distance(new_route, _distances)
                if new_distance < best_distance:
                    best_distance = new_distance
                    best_route = new_route
        return best_route

    # Iterate until a local optimum is reached.
    current_route = initial_route
    while True:
        current_route = two_opt(current_route)
        if calculate_route_distance(current_route, _distances) == calculate_route_distance(initial_route, _distances):
            break

    return current_route
