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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    This function uses a hybrid heuristic approach, combining the nearest neighbor heuristic
    with a local search algorithm.
    """

    # Initialize a random permutation of cities
    np.random.seed(42)  # Set a seed for reproducibility
    current_route = np.random.permutation(np.arange(len(matrix_distances)))

    # Apply nearest neighbor heuristic to find an initial route
    for i in range(len(matrix_distances)):
        current_city = current_route[i]
        next_city = find_nearest_neighbor(current_city, current_route, matrix_distances)
        current_route = np.insert(current_route, i + 1, next_city)

    # Perform local search to refine the route
    current_distance = calculate_route_distance(current_route, matrix_distances)
    while True:
        improved = local_search(current_route, matrix_distances)
        if not improved:
            break
        current_distance = calculate_route_distance(current_route, matrix_distances)

    return current_route


def find_nearest_neighbor(current_city: int, current_route: np.ndarray, matrix_distances: np.ndarray) -> int:
    """Finds the nearest city not already in the route."""
    distances = matrix_distances[current_city]
    unvisited_cities = np.setdiff1d(np.arange(len(matrix_distances)), current_route)
    return unvisited_cities[np.argmin(distances[unvisited_cities])]


def local_search(current_route: np.ndarray, matrix_distances: np.ndarray) -> bool:
    """Performs a local search by swapping two cities in the route."""
    best_distance = calculate_route_distance(current_route, matrix_distances)
    for i in range(len(current_route)):
        for j in range(i + 1, len(current_route)):
            new_route = np.copy(current_route)
            new_route[i], new_route[j] = new_route[j], new_route[i]
            new_distance = calculate_route_distance(new_route, matrix_distances)
            if new_distance < best_distance:
                current_route[:] = new_route[:]
                return True
    return False


def calculate_route_distance(route: np.ndarray, matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return distance
