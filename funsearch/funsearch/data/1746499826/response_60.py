import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(0)

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
    Improved version of find_best_route_v2 using a hybrid approach.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Hybrid approach combining:
    - nearest neighbor heuristic to generate an initial route.
    - 2-opt local search to refine the route.

    Returns:
    tuple[int, ...]: The best route found.
    """

    # Generate an initial route using the nearest neighbor heuristic
    current_city = np.random.randint(len(matrix_distances))
    route = [current_city]
    visited = set([current_city])

    while len(visited) < len(matrix_distances):
        next_city = np.argmin([matrix_distances[current_city][city] for city in range(len(matrix_distances)) if city not in visited])
        route.append(next_city)
        visited.add(next_city)
        current_city = next_city

    # Refine the route using 2-opt local search
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                distance_original = calculate_route_distance(route, matrix_distances)
                route[i:j] = route[j-1:i-1:-1]
                distance_improved = calculate_route_distance(route, matrix_distances)
                if distance_improved < distance_original:
                    break

    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): The route to calculate the distance for.
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    float: The total distance of the route.
    """
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return distance
