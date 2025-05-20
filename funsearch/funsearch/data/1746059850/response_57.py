import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



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
        - Use a combination of nearest neighbor and cheapest insertion.
        - Perform local search using the 2-opt and swap operations.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route with the first city
    route = [0]

    # Nearest neighbor heuristic
    for _ in range(len(distances) - 1):
        current_city = route[-1]
        closest_city = np.argmin(distances[current_city])
        if closest_city not in route:
            route.append(closest_city)

    # Cheapest insertion heuristic
    for _ in range(len(distances) - len(route)):
        min_distance = float('inf')
        for city in range(len(distances)):
            if city not in route:
                distance = distances[route[-1]][city]
                if distance < min_distance:
                    min_distance = distance
                    min_city = city
        route.append(min_city)

    # Local search using 2-opt and swap operations
    for _ in range(100):
        # 2-opt operation
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                distance_diff = distances[route[i]][route[j]] - distances[route[i]][route[j - 1]] - distances[route[i + 1]][route[j]] + distances[route[i + 1]][route[j - 1]]
                if distance_diff < 0:
                    route[i+1:j] = route[j-1:i:-1]

        # Swap operation
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                distance_diff = distances[route[i]][route[j]] - distances[route[i]][route[j - 1]] - distances[route[i + 1]][route[j]] + distances[route[i + 1]][route[j - 1]]
                if distance_diff < 0:
                    route[i], route[j] = route[j], route[i]

    return tuple(route)
