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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    We use a hybrid approach combining nearest neighbor and 2-opt heuristics.
    """

    # Nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    visited = set([start_city])

    while len(visited) < len(distances):
        current_city = route[-1]
        next_city = np.argmin([distance for distance in distances[current_city] if distance not in visited])
        route.append(next_city)
        visited.add(next_city)

    # 2-opt heuristic
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_distance = distances[route[i]][route[j]] + distances[route[(j - 1) % len(route)]][route[(i + 1) % len(route)]] - distances[route[i]][route[(j - 1) % len(route)]] - distances[route[(i + 1) % len(route)]][route[j]]
            if new_distance < distances[route[i]][route[j]]:
                route[i:j+1] = route[j:i-1:-1]

    return tuple(route)
