import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

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
    Improved version of find_best_route_v2 using a hybrid heuristic.

    Hybrid heuristic combines the nearest neighbor strategy to generate an initial solution,
    followed by a local search optimization using the 2-opt neighborhood.

    Args:
        distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        A permutation of cities that minimizes the total route distance.
    """

    # Generate an initial solution using nearest neighbor
    start_city = 0
    route = [start_city]
    unvisited = set(range(len(distances)))
    unvisited.remove(start_city)

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Optimize the solution using local search with 2-opt neighborhood
    for _ in range(100):
        current_distance = calculate_route_distance(route, distances)
        best_distance = current_distance

        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                new_route = route[:i] + route[i+j:j:-1] + route[j+1:]
                new_distance = calculate_route_distance(new_route, distances)

                if new_distance < best_distance:
                    route = new_route
                    best_distance = new_distance

    return tuple(route)


def calculate_route_distance(route: list[int], distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i+1) % len(route)]]
    return total_distance
