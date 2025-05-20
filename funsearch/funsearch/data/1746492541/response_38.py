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
    Improved version of find_best_route_v2.

    Uses a hybrid approach combining nearest neighbor and local search.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    A permutation of cities that minimizes the total route distance.
    """

    # Initialize a random route
    num_cities = len(distances)
    route = np.random.permutation(num_cities)

    # Nearest neighbor heuristic to improve initial route
    current_city = route[0]
    for i in range(1, num_cities):
        closest_city = -1
        min_distance = math.inf
        for j in range(num_cities):
            if j not in route:
                distance = distances[current_city][j]
                if distance < min_distance:
                    closest_city = j
                    min_distance = distance
        route[i] = closest_city
        current_city = closest_city

    # Local search to refine the route
    for i in range(num_cities):
        for j in range(i + 1, num_cities):
            route = local_search(route, distances, i, j)

    return route


def local_search(route: np.ndarray, distances: np.ndarray, i: int, j: int) -> np.ndarray:
    """
    Performs a local search to improve a route by swapping two cities.

    Parameters:
    route (np.ndarray): The current route.
    distances (np.ndarray): A square matrix of distances between cities.
    i (int): The index of the first city to swap.
    j (int): The index of the second city to swap.

    Returns:
    The improved route.
    """

    original_distance = calculate_route_distance(route, distances)

    # Swap the two cities
    route[i], route[j] = route[j], route[i]

    new_distance = calculate_route_distance(route, distances)

    if new_distance < original_distance:
        return route

    # Restore the original route if no improvement was found
    route[i], route[j] = route[j], route[i]
    return route


def calculate_route_distance(route: np.ndarray, distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (np.ndarray): A permutation of cities.
    distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    The total distance of the route.
    """

    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
