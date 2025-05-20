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

    Uses a combination of the nearest neighbor and 2-opt heuristics,
    combined with a local search approach.

    Args:
        distances (np.ndarray): A square matrix of distances between cities.

    Returns:
        A tuple representing the best route.
    """

    # Generate an initial route using the nearest neighbor heuristic
    current_city = np.random.randint(len(distances))
    route = [current_city]
    remaining_cities = set(range(len(distances))) - {current_city}

    while remaining_cities:
        # Find the nearest city to the current city
        nearest_city = min(remaining_cities, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Perform local search using 2-opt and nearest neighbor heuristics
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                # Perform the 2-opt move
                route[i], route[j] = route[j], route[i]

                # Check if the new route is better than the current route
                if calculate_route_distance(route, distances) < calculate_route_distance(route, distances):
                    break
                else:
                    # If not, restore the original route
                    route[i], route[j] = route[j], route[i]

    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Args:
        route (tuple[int, ...]): A tuple representing the route.
        distances (np.ndarray): A square matrix of distances between cities.

    Returns:
        The total distance of the route.
    """

    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
