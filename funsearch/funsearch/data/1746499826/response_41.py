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

    This function combines two heuristics:
        - Nearest neighbor: Start at a random city and iteratively choose the closest unvisited city.
        - 2-opt: Randomly swap two cities in the route and update the best route if it results in a shorter distance.

    Returns:
    A permutation of cities that minimizes the total route distance.
    """

    # Initialize the route using the nearest neighbor heuristic
    num_cities = len(matrix_distances)
    current_city = np.random.randint(num_cities)
    route = [current_city]

    # Add the remaining cities using the nearest neighbor heuristic
    remaining_cities = set(range(num_cities))
    remaining_cities.remove(current_city)
    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Perform 2-opt operations to improve the route
    for _ in range(100):
        i, j = np.random.randint(num_cities, size=2)
        if calculate_route_distance(route[:i] + route[j:i:-1] + route[j+1:], matrix_distances) < calculate_route_distance(route, matrix_distances):
            route = route[:i] + route[j:i:-1] + route[j+1:]

    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities.
    matrix_distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    The total distance of the route.
    """

    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i+1) % len(route)]]

    return total_distance
