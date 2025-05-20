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

    This function uses a hybrid approach combining the nearest neighbor and 2-opt heuristics.
    """

    # Initial route using nearest neighbor heuristic
    start_city = np.random.randint(len(distances))
    route = [start_city]
    unvisited = set(range(len(distances)))
    unvisited.remove(start_city)

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda c: distances[current_city][c])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Improve the route using 2-opt heuristic
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            if distances[route[i]][route[j]] < distances[route[(i - 1) % len(route)]][route[j]]:
                route[i:j+1] = route[j:i:-1]

    return tuple(route)
