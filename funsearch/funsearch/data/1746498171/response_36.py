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
    - Local search

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a random seed for reproducibility
    np.random.seed(42)

    # Initialize the starting city
    current_city = np.random.randint(len(distances))

    # Create an empty list to store the route
    route = [current_city]

    # Create a list of unvisited cities
    unvisited_cities = list(range(len(distances)))
    unvisited_cities.remove(current_city)

    # Iterate until all cities have been visited
    while unvisited_cities:
        # Get the nearest unvisited city
        nearest_city = min(unvisited_cities, key=lambda city: distances[current_city][city])

        # Add the nearest city to the route
        route.append(nearest_city)

        # Remove the nearest city from the list of unvisited cities
        unvisited_cities.remove(nearest_city)

        # Update the current city
        current_city = nearest_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
