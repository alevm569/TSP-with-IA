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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v0`, using a hybrid approach combining nearest neighbor and 2-opt heuristics.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: The best route as a permutation of city indices.
    """

    # Initialize the route using nearest neighbor
    start_city = np.random.randint(len(distances))
    route = [start_city]
    unvisited = set(range(len(distances))) - {start_city}

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Perform 2-opt local search to refine the route
    for _ in range(100):
        i, j = np.random.randint(0, len(route), 2)
        if i > j:
            i, j = j, i
        route = two_opt(route, distances)

    return tuple(route)

def two_opt(route: list[int], distances: np.ndarray) -> list[int]:
    """
    Performs a 2-opt move on the given route.

    Parameters:
    route (list[int]): The current route.
    distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    list[int]: The modified route after the 2-opt move.
    """

    best_distance = calculate_route_distance(route, distances)

    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            new_route = route[:i] + route[i+j:i:-1] + route[j+1:]
            new_distance = calculate_route_distance(new_route, distances)

            if new_distance < best_distance:
                best_distance = new_distance
                best_route = new_route

    return best_route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of city indices.
    distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    float: The total distance of the route.
    """

    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i+1) % len(route)]]

    return total_distance
