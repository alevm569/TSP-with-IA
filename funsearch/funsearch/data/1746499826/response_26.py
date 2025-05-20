import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

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
    Improved version of find_best_route_v2, using a hybrid heuristic.
    """
    # Create a set of unvisited cities
    unvisited_cities = set(range(len(matrix_distances)))

    # Start from an initial city
    current_city = np.random.choice(list(unvisited_cities))
    route = [current_city]

    # Repeat until all cities are visited
    while unvisited_cities:
        # Find the nearest unvisited city
        nearest_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])

        # Add the nearest city to the route
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

        # Update the current city
        current_city = nearest_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
