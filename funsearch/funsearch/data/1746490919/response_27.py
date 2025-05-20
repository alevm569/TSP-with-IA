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

def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2.
    Uses a hybrid approach combining nearest neighbor and 2-opt heuristics.
    """

    # Initialize a random starting city
    start_city = np.random.randint(len(matrix_distances))

    # Nearest neighbor heuristic
    route = [start_city]
    unvisited_cities = set(range(len(matrix_distances)))
    unvisited_cities.remove(start_city)

    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # 2-opt heuristic
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_original = matrix_distances[route[i]][route[j]]
            distance_reversed = matrix_distances[route[i]][route[(j - 1)]] + matrix_distances[route[(j - 1)]][route[j]] - distance_original
            if distance_reversed < distance_original:
                route[i+1:j] = route[j-1:i:-1]

    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i+1) % len(route)]]
    return total_distance
