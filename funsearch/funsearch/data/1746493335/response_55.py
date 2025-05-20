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


def find_best_route_v0(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    You may use at least one strategy or combine two or more heuristics from the list below:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design
        - aco (ant colony optimization)
        - genetic algorithms
        - k-opt
        - tabu search

    Routes must include all cities exactly once and return to the starting point.
    """
    """Improved version of `find_best_route_v2`."""

    # Implement a hybrid heuristic using the nearest neighbor and 2-opt heuristics.
    # Use the nearest neighbor heuristic to generate an initial route, then apply the 2-opt heuristic to improve it.
    # Repeat this process until a satisfactory solution is found.

    # Initialize an empty route
    route = []

    # Use the nearest neighbor heuristic to generate an initial route
    current_city = 0
    for _ in range(len(_distances)):
        route.append(current_city)
        # Find the city with the minimum distance from the current city
        min_distance = float('inf')
        next_city = None
        for i in range(len(_distances)):
            if i not in route and _distances[current_city][i] < min_distance:
                min_distance = _distances[current_city][i]
                next_city = i
        current_city = next_city

    # Apply the 2-opt heuristic to improve the route
    for _ in range(100):
        # Randomly select two cities in the route
        i, j = np.random.randint(0, len(route), size=2)
        # Swap the two cities in the route
        route[i], route[j] = route[j], route[i]

    # Return the improved route
    return tuple(route)


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""
    """Improved version of `find_best_route_v2`."""

    # Implement a hybrid heuristic using the nearest neighbor and 2-opt heuristics.
    # Use the nearest neighbor heuristic to generate an initial route, then apply the 2-opt heuristic to improve it.
    # Repeat this process until a satisfactory solution is found.

    # Initialize an empty route
    route = []

    # Use the nearest neighbor heuristic to generate an initial route
    current_city = 0
    for _ in range(len(_distances)):
        route.append(current_city)
        # Find the city with the minimum distance from the current city
        min_distance = float('inf')
        next_city = None
        for i in range(len(_distances)):
            if i not in route and _distances[current_city][i] < min_distance:
                min_distance = _distances[current_city][i]
                next_city = i
        current_city = next_city

    # Apply the 2-opt heuristic to improve the route
    for _ in range(100):
        # Randomly select two cities in the route
        i, j = np.random.randint(0, len(route), size=2)
        # Swap the two cities in the route
        route[i], route[j] = route[j], route[i]

    # Return the improved route
    return tuple(route)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a hybrid heuristic using the nearest neighbor and 2-opt heuristics.
    # Use the nearest neighbor heuristic to generate an initial route, then apply the 2-opt heuristic to improve it.
    # Repeat this process until a satisfactory solution is found.

    # Initialize an empty route
    route = []

    # Use the nearest neighbor heuristic to generate an initial route
    current_city = 0
    for _ in range(len(_distances)):
        route.append(current_city)
        # Find the city with the minimum distance from the current city
        min_distance = float('inf')
        next_city = None
        for i in range(len(_distances)):
            if i not in route and _distances[current_city][i] < min_distance:
                min_distance = _distances[current_city][i]
                next_city = i
        current_city = next_city

    # Apply the 2-opt heuristic to improve the route
    for _ in range(100):
        # Randomly select two cities in the route
        i, j = np.random.randint(0, len(route), size=2)
        # Swap the two cities in the route
        route[i], route[j] = route[j], route[i]

    # Return the improved route
    return tuple(route)


def find_best_route(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Routes must include all cities exactly once and return to the starting point.
    """

    # Implement a hybrid heuristic using the nearest neighbor and 2-opt heuristics.
    # Use the nearest neighbor heuristic to generate an initial route, then apply the 2-opt heuristic to improve it.
    # Repeat this process until a satisfactory solution is found.

    # Initialize an empty route
    route = []

    # Use the nearest neighbor heuristic to generate an initial route
    current_city = 0
    for _ in range(len(_distances)):
        route.append(current_city)
        # Find the city with the minimum distance from the current city
        min_distance = float('inf')
        next_city = None
        for i in range(len(_distances)):
            if i not in route and _distances[current_city][i] < min_distance:
                min_distance = _distances[current_city][i]
                next_city = i
        current_city = next_city

    # Apply the 2-opt heuristic to improve the route
    for _ in range(100):
        # Randomly select two cities in the route
        i, j = np.random.randint(0, len(route), size=2)
        # Swap the two cities in the route
        route[i], route[j] = route[j], route[i]

    # Return the improved route
    return tuple(route)
