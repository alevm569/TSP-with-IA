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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a hybrid heuristic.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    This function implements a hybrid heuristic that combines the nearest neighbor and cheapest insertion heuristics.
    """

    # Initialize a random starting city
    start_city = np.random.randint(len(matrix_distances))

    # Initialize an empty route
    route = [start_city]

    # Create a set of unvisited cities
    unvisited_cities = set(range(len(matrix_distances)))
    unvisited_cities.remove(start_city)

    # Iterate until all cities have been visited
    while unvisited_cities:
        # Get the current city
        current_city = route[-1]

        # Find the nearest unvisited city
        nearest_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])

        # Find the cheapest insertion city
        cheapest_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city] + matrix_distances[city][nearest_city])

        # Add the cheapest insertion city to the route
        route.append(cheapest_city)
        unvisited_cities.remove(cheapest_city)

    # Add the starting city back to the route
    route.append(start_city)

    return tuple(route)
