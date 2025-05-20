import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Finds the best route using a hybrid heuristic approach.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: The best route as a tuple of city indices.
    """

    # Use a combination of nearest neighbor and cheapest insertion heuristics
    start_city = 0
    route = [start_city]
    unvisited_cities = set(range(1, len(matrix_distances)))

    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])
        cheapest_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city] + matrix_distances[city][start_city])

        # Choose the city with the lowest cost (nearest neighbor or cheapest insertion)
        if np.random.rand() < 0.5:
            route.append(nearest_city)
        else:
            route.append(cheapest_city)

        unvisited_cities.remove(nearest_city)

    # Return to the starting city
    route.append(start_city)

    return tuple(route)
