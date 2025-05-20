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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
    - 2-opt: Randomly swap two cities in the route and choose the best route.
    - Tabu search: Explore nearby routes by swapping two cities, but avoid repeating recently visited routes.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Generate a random initial route
    num_cities = distances.shape[0]
    initial_route = np.random.permutation(num_cities)

    # Create a tabu search object
    tabu_size = 10  # Size of the tabu list
    tabu_list = []

    # Perform tabu search
    best_route = initial_route
    best_distance = calculate_route_distance(best_route, distances)

    for _ in range(1000):  # Number of iterations
        # Generate a new route by swapping two random cities
        route = np.copy(best_route)
        i, j = np.random.choice(num_cities, 2, replace=False)
        route[i], route[j] = route[j], route[i]

        # Check if the new route is valid and better than the current best route
        if is_valid_route(route) and calculate_route_distance(route, distances) < best_distance:
            best_route = route
            best_distance = calculate_route_distance(best_route, distances)

        # Add the current route to the tabu list
        tabu_list.append(best_route)
        if len(tabu_list) > tabu_size:
            tabu_list.pop(0)

    return best_route


def is_valid_route(route: tuple[int, ...]) -> bool:
    """Checks if a route includes all cities exactly once."""
    return len(set(route)) == len(route) and route[0] == route[-1]


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i], route[(i + 1) % len(route)]]
    return total_distance
