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

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
        - Nearest neighbor
        - Cheapest insertion

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the starting city and the list of unvisited cities
    start_city = 0
    unvisited_cities = list(range(1, len(distances)))

    # Initialize the best route and its distance
    best_route = [start_city]
    best_distance = 0

    # Start the nearest neighbor algorithm
    current_city = start_city
    while unvisited_cities:
        # Find the closest unvisited city
        nearest_city = min(unvisited_cities, key=lambda city: distances[current_city][city])

        # Add the nearest city to the route
        best_route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

        # Calculate the distance between the current and nearest cities
        best_distance += distances[current_city][nearest_city]

        # Update the current city
        current_city = nearest_city

    # Close the route by returning to the starting city
    best_route.append(start_city)
    best_distance += distances[current_city][start_city]

    return tuple(best_route)
