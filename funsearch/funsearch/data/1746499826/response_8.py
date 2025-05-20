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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v1 with hybrid heuristic.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Hybrid heuristic combining nearest neighbor, cheapest insertion, and 2-opt.
    """
    # Initialize a random starting city
    current_city = np.random.randint(len(distances))

    # Build an empty route
    route = []

    # Use nearest neighbor to find the next city
    while len(route) < len(distances):
        route.append(current_city)
        # Find the city with the shortest distance from the current city
        nearest_city = np.argmin(distances[current_city])
        current_city = nearest_city

    # Use cheapest insertion to insert the remaining cities
    while len(route) < len(distances):
        # Find the city that can be added with the lowest additional distance
        cheapest_city = np.argmin(np.min(distances[route], axis=0))
        route.append(cheapest_city)

    # Use 2-opt to improve the route
    funsearch.optimize.two_opt(route, distances)

    return tuple(route)
