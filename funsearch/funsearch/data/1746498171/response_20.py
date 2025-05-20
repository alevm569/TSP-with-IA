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
    Improved version of the find_best_route_v2 function using hybrid heuristics.

    Args:
        distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        tuple[int, ...]: A permutation of cities that minimizes the total route distance, including the return to the starting city.
    """

    # Use hybrid heuristics:
    # - Start with nearest neighbor to generate an initial route.
    # - Use cheapest insertion to refine the route by adding cities with the lowest distances.
    # - Perform local search to optimize the route by iteratively swapping two cities and checking if it improves the total distance.

    # Initial route using nearest neighbor
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(1, len(distances)))

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Refine route using cheapest insertion
    for _ in range(len(distances)):
        min_distance = float('inf')
        for city in remaining_cities:
            distance = distances[route[-1]][city]
            if distance < min_distance:
                min_distance = distance
                best_city = city
        route.append(best_city)
        remaining_cities.remove(best_city)

    # Optimize route using local search
    for _ in range(100):
        i, j = np.random.randint(0, len(route), size=2)
        route[i], route[j] = route[j], route[i]
        if calculate_route_distance(route, distances) < calculate_route_distance(route[:-1], distances):
            pass
        else:
            route[i], route[j] = route[j], route[i]

    return tuple(route)
