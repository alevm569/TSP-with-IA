import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(0)  # Set random seed for reproducibility

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
    Improved version of find_best_route_v0, using a combination of nearest neighbor and local search.

    Args:
        matrix_distances (np.ndarray): A square matrix of distances between cities.

    Returns:
        A permutation of cities that minimizes the total route distance.
    """

    # Initialize a random starting city
    current_city = np.random.randint(len(matrix_distances))

    # Create an empty list to store the route
    route = []

    # Build the route using the nearest neighbor heuristic
    while len(route) < len(matrix_distances):
        route.append(current_city)

        # Find the city with the minimum distance from the current city
        next_city = np.argmin(matrix_distances[current_city])

        # Ensure we don't visit the same city again
        while next_city in route:
            next_city = np.argmin(matrix_distances[current_city] + 1000)

        # Move to the next city
        current_city = next_city

    # Close the route by adding the starting city
    route.append(route[0])

    # Perform local search to optimize the route
    for i in range(100):
        route = local_search(route, matrix_distances)

    return tuple(route)


def local_search(route: list[int], matrix_distances: np.ndarray) -> list[int]:
    """
    Performs local search on a route to improve its quality.

    Args:
        route (list[int]): The current route.
        matrix_distances (np.ndarray): A square matrix of distances between cities.

    Returns:
        The improved route.
    """

    # Iterate over all pairs of cities in the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Swap the two cities in the route
            route[i], route[j] = route[j], route[i]

            # Calculate the total distance of the new route
            total_distance = calculate_route_distance(route, matrix_distances)

            # If the new route is better, keep it
            if total_distance < calculate_route_distance(route, matrix_distances):
                return route

            # Swap the two cities back
            route[i], route[j] = route[j], route[i]

    return route


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Args:
        route (tuple[int, ...]): A permutation of cities.
        matrix_distances (np.ndarray): A square matrix of distances between cities.

    Returns:
        The total distance of the route.
    """

    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
