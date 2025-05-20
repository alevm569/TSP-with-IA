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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Use a hybrid approach that combines the following heuristics:

    - **Nearest neighbor:** Start from an initial city and iteratively choose the closest unvisited city as the next destination.
    - **2-opt:** Randomly select two edges in the route and reverse their order to potentially improve the distance.
    - **Local search:** Apply the 2-opt operation repeatedly until no further improvements are found.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a random seed for reproducibility
    np.random.seed(0)

    # Generate an initial route using the nearest neighbor heuristic
    num_cities = len(matrix_distances)
    current_city = np.random.randint(num_cities)
    route = [current_city]

    for _ in range(num_cities - 1):
        # Find the closest unvisited city
        min_distance = float('inf')
        for city in range(num_cities):
            if city not in route:
                distance = matrix_distances[current_city][city]
                if distance < min_distance:
                    min_distance = distance
                    next_city = city

        # Add the next city to the route
        route.append(next_city)
        current_city = next_city

    # Perform local search using the 2-opt operation
    for _ in range(100):
        # Randomly select two edges
        i, j = np.random.randint(num_cities, size=2)
        route[i], route[j] = route[j], route[i]

        # Check if the route distance is improved
        if calculate_route_distance(route, matrix_distances) < calculate_route_distance(route, matrix_distances):
            pass
        else:
            # If not improved, reverse the edges back
            route[i], route[j] = route[j], route[i]

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
