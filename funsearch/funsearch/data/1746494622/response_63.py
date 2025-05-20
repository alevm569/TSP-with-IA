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

    Heuristics used:
    - 2-opt: Swap two edges in the current route to find a shorter route.
    - Tabu search: Avoid exploring previously visited states to avoid getting stuck in local minima.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route and best route
    num_cities = matrix_distances.shape[0]
    route = np.random.permutation(num_cities)
    best_route = route.copy()
    best_distance = calculate_route_distance(best_route, matrix_distances)

    # Initialize the tabu list
    tabu_list = []

    # Tabu search iterations
    for _ in range(1000):

        # Generate a new route by swapping two edges
        i, j = np.random.randint(num_cities, size=2)
        route[i], route[j] = route[j], route[i]

        # Calculate the distance of the new route
        distance = calculate_route_distance(route, matrix_distances)

        # Update the best route if necessary
        if distance < best_distance:
            best_route = route.copy()
            best_distance = distance

        # Add the current route to the tabu list
        tabu_list.append(route.copy())

        # Remove the oldest route from the tabu list if it reaches a certain size
        if len(tabu_list) > 100:
            tabu_list.pop(0)

        # Remove the current route from the tabu list if it has been visited before
        if route in tabu_list:
            route = best_route.copy()

    return best_route


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return distance
