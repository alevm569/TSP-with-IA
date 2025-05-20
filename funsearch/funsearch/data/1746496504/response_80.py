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

    You may use at least one strategy or combine two or more heuristics from the list below:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design
        - aco (ant colony optimization)
        - genetic algorithms
        - k-opt
        - tabu search

    Routes must include all cities exactly once and return to the starting point.
    """

    # Use a hybrid approach combining local search and 2-opt heuristics
    start_city = 0
    current_city = start_city
    route = [current_city]

    # Perform local search to find the best starting route
    for _ in range(len(distances)):
        best_distance = np.inf
        for next_city in range(len(distances)):
            if next_city not in route:
                distance = distances[current_city][next_city]
                if distance < best_distance:
                    best_distance = distance
                    best_city = next_city

        route.append(best_city)
        current_city = best_city

    # Perform 2-opt optimization to refine the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance = distances[route[i]][route[j]]
            if distance < distances[route[i]][route[(j + 1) % len(route)]] + distances[route[j]][route[(i + 1) % len(route)]]:
                route[i], route[j] = route[j], route[i]

    return tuple(route)
