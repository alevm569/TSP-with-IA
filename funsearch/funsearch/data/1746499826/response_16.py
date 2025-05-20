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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
    - Nearest neighbor: Start from an initial city and iteratively find the nearest unvisited city.
    - Cheapest insertion: Find the city that minimizes the total distance when added to the route.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize starting city
    current_city = 0
    route = [current_city]

    # Create a set of unvisited cities
    unvisited_cities = set(range(len(distances)))
    unvisited_cities.remove(current_city)

    # Use nearest neighbor and cheapest insertion heuristics
    while unvisited_cities:
        # Find the nearest unvisited city
        nearest_city = min(unvisited_cities, key=lambda c: distances[current_city][c])

        # Find the city that minimizes total distance when added
        cheapest_city = min(unvisited_cities, key=lambda c: distances[current_city][c] + distances[c][nearest_city])

        # Add the cheapest city to the route
        route.append(cheapest_city)
        unvisited_cities.remove(cheapest_city)

        # Update current city
        current_city = cheapest_city

    # Close the route by adding the starting city again
    route.append(route[0])

    return tuple(route)
