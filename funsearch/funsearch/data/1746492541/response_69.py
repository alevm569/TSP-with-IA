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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Uses a hybrid approach combining local search and the 2-opt heuristic.
    """

    # Generate an initial random route
    num_cities = len(distances)
    route = np.random.permutation(num_cities)

    # Local search with 2-opt neighborhood operator
    for _ in range(100):  # Run for 100 iterations
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                # Calculate the distance difference if cities i and j are swapped
                distance_difference = distances[route[i]][route[j]] + distances[route[(j - 1) % num_cities]][route[(i + 1) % num_cities]] - distances[route[i]][route[(j - 1) % num_cities]] - distances[route[j]][route[(i + 1) % num_cities]]

                # If swapping improves the route distance, swap the cities
                if distance_difference < 0:
                    route[i], route[j] = route[j], route[i]

    return route
