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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2, using a hybrid approach combining nearest neighbor, cheapest insertion, and local search.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Initialize a random starting city
    start_city = np.random.randint(len(matrix_distances))

    # Use nearest neighbor to find an initial route
    route = [start_city]
    visited = set([start_city])
    while len(visited) < len(matrix_distances):
        current_city = route[-1]
        nearest_city = np.argmin([matrix_distances[current_city][j] for j in range(len(matrix_distances)) if j not in visited])
        route.append(nearest_city)
        visited.add(nearest_city)

    # Use cheapest insertion to refine the route
    for i in range(len(route)):
        best_insertion = np.argmin([calculate_route_distance(route[:i] + [j] + route[i:], matrix_distances) for j in range(len(matrix_distances)) if j not in route])
        route.insert(i, best_insertion)

    # Use local search to improve the route
    for _ in range(100):
        best_route = route
        best_distance = calculate_route_distance(route, matrix_distances)
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = route[:i] + route[j:i:-1] + route[j + 1:]
                new_distance = calculate_route_distance(new_route, matrix_distances)
                if new_distance < best_distance:
                    best_route = new_route
                    best_distance = new_distance
        route = best_route

    return route


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities.
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    float: The total distance of the route.
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
