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
    - Use a hybrid heuristic that combines:
        - Nearest neighbor for initial route construction.
        - Cheapest insertion for adding subsequent cities.
        - Local search to refine the route.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a random starting city
    start_city = np.random.randint(len(distances))

    # Nearest neighbor heuristic for initial route construction
    route = [start_city]
    unvisited = set(range(len(distances)))
    unvisited.remove(start_city)
    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Cheapest insertion heuristic for adding subsequent cities
    while unvisited:
        best_city = None
        best_distance = float('inf')
        for city in unvisited:
            distance = distances[route[-1]][city]
            if distance < best_distance:
                best_distance = distance
                best_city = city
        route.append(best_city)
        unvisited.remove(best_city)

    # Local search to refine the route
    for _ in range(10):
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                distance_difference = distances[route[i]][route[j]] - distances[route[i]][route[(j + 1) % len(route)]] - distances[route[(j - 1) % len(route)]][route[j]]
                if distance_difference < 0:
                    route[i], route[j] = route[j], route[i]

    return tuple(route)
