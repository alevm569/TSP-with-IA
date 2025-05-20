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
    Improved version of find_best_route_v2 using hybrid heuristics.

    Parameters:
        matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        A permutation of cities that minimizes the total route distance.
    """

    # Initialize a set of unvisited cities
    unvisited_cities = set(range(len(matrix_distances)))

    # Choose a starting city
    current_city = np.random.choice(list(unvisited_cities))
    unvisited_cities.remove(current_city)

    # Initialize the route
    route = [current_city]

    # Iterate until all cities are visited
    while unvisited_cities:
        # Find the city with the minimum distance to the current city
        next_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])
        unvisited_cities.remove(next_city)

        # Add the next city to the route
        route.append(next_city)

        # Update the current city
        current_city = next_city

    # Close the route by returning to the starting city
    route.append(route[0])

    return tuple(route)
