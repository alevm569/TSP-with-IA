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
    Improved version of the find_best_route function.

    Uses a hybrid approach combining the nearest neighbor and local search heuristics.
    """

    # Initialize a random starting city
    current_city = np.random.randint(len(matrix_distances))

    # Create an empty route
    route = [current_city]

    # Create a set of unvisited cities
    unvisited_cities = set(range(len(matrix_distances)))
    unvisited_cities.remove(current_city)

    # Find the nearest neighbor for each unvisited city
    for _ in range(len(matrix_distances) - 1):
        nearest_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)
        current_city = nearest_city

    # Close the route by returning to the starting city
    route.append(route[0])

    # Perform local search to refine the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:]
            new_route[i], new_route[j] = new_route[j], new_route[i]
            if calculate_route_distance(new_route, matrix_distances) < calculate_route_distance(route, matrix_distances):
                route = new_route

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.
    """
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return distance
