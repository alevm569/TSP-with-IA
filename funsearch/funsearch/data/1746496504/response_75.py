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

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    This function uses a hybrid approach combining the nearest neighbor and cheapest insertion heuristics.
    """

    # Initialize current city and route
    current_city = 0
    route = [current_city]

    # Create a set of unvisited cities
    unvisited_cities = set(range(len(distances)))

    # Iterate until all cities are visited
    while len(unvisited_cities) > 0:
        # Find the nearest unvisited city
        nearest_city = min(unvisited_cities, key=lambda city: distances[current_city][city])

        # Find the cheapest insertion city
        cheapest_city = min(unvisited_cities, key=lambda city: distances[route[-1]][city])

        # Add the cheapest insertion city to the route
        route.append(cheapest_city)
        unvisited_cities.remove(cheapest_city)

        # Update the current city
        current_city = cheapest_city

    # Return the route, including the return to the starting city
    return tuple(route + [route[0]])
