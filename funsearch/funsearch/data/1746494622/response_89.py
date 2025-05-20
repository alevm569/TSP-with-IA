import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Implement a new version of the find_best_route function that solves the TSP problem using a hybrid heuristic approach.
    # This approach combines multiple heuristics, such as nearest neighbor, 2-opt, and local search, to generate a high-quality solution.

    # Initialization
    num_cities = len(_distances)
    current_city = np.random.randint(num_cities)
    route = [current_city]

    # Generate a preliminary route using the nearest neighbor heuristic
    for _ in range(num_cities - 1):
        current_city = find_nearest_neighbor(current_city, _distances, route)
        route.append(current_city)

    # Perform local search to refine the route
    for _ in range(num_cities):
        best_route = route
        best_distance = calculate_route_distance(route, _distances)

        # Try swapping two cities in the route and check if it improves the distance
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                new_route = swap_cities(route, i, j)
                new_distance = calculate_route_distance(new_route, _distances)

                if new_distance < best_distance:
                    best_distance = new_distance
                    best_route = new_route

        route = best_route

    # Return the optimal route
    return tuple(route)

def find_nearest_neighbor(current_city: int, distances: np.ndarray, route: list) -> int:
    """Finds the nearest city not already in the route."""
    min_distance = np.inf
    nearest_city = None

    for i in range(len(distances)):
        if i not in route:
            distance = distances[current_city][i]
            if distance < min_distance:
                min_distance = distance
                nearest_city = i

    return nearest_city

def swap_cities(route: list, i: int, j: int) -> list:
    """Swaps two cities in the route."""
    route[i], route[j] = route[j], route[i]
    return route

def calculate_route_distance(route: list, distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0

    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
