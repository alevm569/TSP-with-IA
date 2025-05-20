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
    Finds the best route using a combination of the nearest neighbor and 2-opt heuristics.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Initialize the route using the nearest neighbor heuristic.
    route = nearest_neighbor(distances)

    # Apply 2-opt optimization to improve the route.
    route = two_opt(route, distances)

    return route


def nearest_neighbor(distances: np.ndarray) -> tuple[int, ...]:
    """
    Finds the best route using the nearest neighbor heuristic.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Initialize the route with the first city.
    route = [0]

    # Visit each city in the order of their distance to the last city in the route.
    for _ in range(len(distances) - 1):
        current_city = route[-1]
        next_city = np.argmin(distances[current_city])
        route.append(next_city)

    return tuple(route)


def two_opt(route: tuple[int, ...], distances: np.ndarray) -> tuple[int, ...]:
    """
    Improves the route using the 2-opt heuristic.

    Parameters:
    route (tuple[int, ...]): The current route.
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: The improved route.
    """

    # Iterate over all pairs of cities in the route.
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Calculate the distance of the current route.
            current_distance = calculate_route_distance(route, distances)

            # Reverse the order of the two cities in the route.
            route = route[:i] + route[j:i:-1] + route[j+1:]

            # Calculate the distance of the improved route.
            improved_distance = calculate_route_distance(route, distances)

            # If the improved route is shorter, keep it.
            if improved_distance < current_distance:
                break

    return route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): The route.
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    float: The total distance of the route.
    """

    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
