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

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristic:
    - Hybrid heuristic using a combination of nearest neighbor and 2-opt operations.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a random route
    num_cities = distances.shape[0]
    route = np.random.permutation(num_cities)

    # Perform hybrid heuristic
    while True:
        # Nearest neighbor operation
        nearest_city = np.argmin(distances[route[-1]])
        if nearest_city not in route:
            route = np.append(route, nearest_city)

        # 2-opt operation
        best_distance = calculate_route_distance(route, distances)
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                new_route = route.copy()
                new_route[i:j+1] = new_route[j:i:-1]
                new_distance = calculate_route_distance(new_route, distances)
                if new_distance < best_distance:
                    route = new_route
                    best_distance = new_distance

        # Check if the route is valid
        if len(set(route)) == num_cities:
            break

    return route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities.
    distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    float: The total distance of the route.
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i+1) % len(route)]]
    return total_distance
