import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

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
    Improved version of the find_best_route function using a hybrid approach.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    This function uses a combination of the nearest neighbor heuristic, local search, and 2-opt heuristics to find the best route.
    """

    # Initialize a random route
    num_cities = matrix_distances.shape[0]
    route = np.random.permutation(num_cities)

    # Nearest neighbor heuristic to initialize the route
    current_city = 0
    while len(route) < num_cities:
        nearest_city = np.argmin(matrix_distances[current_city])
        if nearest_city not in route:
            route = np.append(route, nearest_city)
            current_city = nearest_city

    # Local search to improve the route
    for _ in range(100):
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                # Swap two cities in the route
                route[i], route[j] = route[j], route[i]

                # Calculate the total distance of the new route
                total_distance = calculate_route_distance(route, matrix_distances)

                # If the new route is better, keep it
                if total_distance < calculate_route_distance(route, matrix_distances):
                    break

    # 2-opt heuristic to further improve the route
    for _ in range(100):
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                route[i:j+1] = route[j:i:-1]

                # Calculate the total distance of the new route
                total_distance = calculate_route_distance(route, matrix_distances)

                # If the new route is better, keep it
                if total_distance < calculate_route_distance(route, matrix_distances):
                    break

    return route

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    float: The total distance of the route
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
