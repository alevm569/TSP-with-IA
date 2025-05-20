import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

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
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    This version combines the nearest neighbor heuristic for initial route building,
    cheapest insertion for route optimization, and local search for further improvement.
    """

    # Initialize a random starting city
    start_city = np.random.randint(len(distances))

    # Build an initial route using the nearest neighbor heuristic
    route = [start_city]
    unvisited = set(range(len(distances)))
    unvisited.remove(start_city)

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Optimize the route using the cheapest insertion heuristic
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            if distances[route[i]][route[j]] > distances[route[i]][route[(j + 1) % len(route)]] + distances[route[(j + 1) % len(route)]][route[j]]:
                route[i], route[j] = route[j], route[i]

    # Improve the route using local search
    for i in range(len(route)):
        best_distance = calculate_route_distance(route, distances)

        for j in range(len(route)):
            if j != i:
                route[i], route[j] = route[j], route[i]
                distance = calculate_route_distance(route, distances)

                if distance < best_distance:
                    best_distance = distance

        route[i] = route[j]

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += distances[route[i]][route[(i + 1) % len(route)]]
    return distance
