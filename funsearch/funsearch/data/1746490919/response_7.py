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
    """Improved version of `find_best_route_v1`."""

    # Hybrid heuristic combining nearest neighbor and k-opt
    def hybrid_heuristic(distances):
        # Start with nearest neighbor solution
        current_city = 0
        route = [current_city]
        unvisited = set(range(len(distances)))
        unvisited.remove(current_city)

        while unvisited:
            # Find the nearest unvisited city
            nearest_city = min(unvisited, key=lambda city: distances[current_city][city])
            route.append(nearest_city)
            unvisited.remove(nearest_city)
            current_city = nearest_city

        # Apply k-opt to improve the route
        k = 2  # Number of cities to swap in each k-opt move
        for _ in range(10):  # Number of k-opt iterations
            funsearch.k_opt(route, distances, k)

        return route

    # Use the hybrid heuristic to find the best route
    best_route = hybrid_heuristic(_distances)

    return best_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
