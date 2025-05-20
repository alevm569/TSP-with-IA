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

    Use a combination of the following heuristics:

    - Nearest neighbor: Start at an arbitrary city and iteratively select the city with the minimum distance to the current city.
    - Cheapest insertion: Select the city that, when added to the route, results in the lowest total distance.
    - Local search: Apply a neighborhood operator to explore nearby solutions and potentially improve the current best route.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a random starting city
    start_city = np.random.randint(len(matrix_distances))

    # Create an empty route
    route = [start_city]

    # Use the nearest neighbor heuristic to find the remaining cities
    unvisited_cities = set(range(len(matrix_distances)))
    unvisited_cities.remove(start_city)
    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Perform local search to improve the route
    for i in range(100):
        route = local_search(route, matrix_distances)

    # Return the route
    return tuple(route)


def local_search(route: list[int], matrix_distances: np.ndarray) -> list[int]:
    """
    Apply a neighborhood operator to explore nearby solutions and potentially improve the current best route.
    """

    # Randomly swap two cities in the route
    i, j = np.random.randint(len(route), size=2)
    route[i], route[j] = route[j], route[i]

    return route


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of the given route.
    """

    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]

    return distance
