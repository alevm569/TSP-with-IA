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

def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Finds the best route using a hybrid heuristic that combines the nearest neighbor and cheapest insertion strategies.

    Parameters:
        matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        A permutation of cities that minimizes the total route distance, including the return to the starting city.
    """

    # Initialize current route and visited cities
    current_city = 0
    visited = set([current_city])
    route = [current_city]

    # Perform hybrid heuristic
    while len(visited) < len(matrix_distances):
        # Nearest neighbor strategy: Find the closest unvisited city
        distances = matrix_distances[current_city]
        nearest_city = np.argmin(distances[np.isin(np.arange(len(distances)), visited, invert=True)])

        # Cheapest insertion strategy: Find the cheapest city to add to the route
        cheapest_city = np.argmin([calculate_route_distance(route + [city], matrix_distances) for city in range(len(matrix_distances)) if city not in visited])

        # Add the cheapest city to the route and mark it as visited
        route.append(cheapest_city)
        visited.add(cheapest_city)

        # Update current city
        current_city = cheapest_city

    # Return to starting city
    route.append(route[0])

    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
