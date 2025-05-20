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
    Improved version of find_best_route_v2.

    Uses a hybrid approach combining nearest neighbor and local search.
    """

    # Initialize a random route
    np.random.seed(42)
    random_route = np.random.permutation(np.arange(len(matrix_distances)))

    # Use nearest neighbor to find an initial good solution
    best_route = nearest_neighbor(matrix_distances)

    # Perform local search to improve the solution
    best_distance = calculate_route_distance(best_route, matrix_distances)
    for _ in range(100):
        current_route = local_search(best_route, matrix_distances)
        current_distance = calculate_route_distance(current_route, matrix_distances)
        if current_distance < best_distance:
            best_route = current_route
            best_distance = current_distance

    return best_route


def nearest_neighbor(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """Finds a route using the nearest neighbor heuristic."""
    num_cities = len(matrix_distances)
    unvisited_cities = set(range(num_cities))
    current_city = np.random.choice(num_cities)
    route = [current_city]

    while len(unvisited_cities) > 0:
        unvisited_cities.remove(current_city)
        next_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(next_city)
        current_city = next_city

    return tuple(route)


def local_search(route: tuple[int, ...], matrix_distances: np.ndarray) -> tuple[int, ...]:
    """Performs local search to improve a given route."""
    best_route = route
    best_distance = calculate_route_distance(best_route, matrix_distances)

    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:i] + (route[j], route[i],) + route[j+1:]
            new_distance = calculate_route_distance(new_route, matrix_distances)
            if new_distance < best_distance:
                best_route = new_route
                best_distance = new_distance

    return best_route


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return distance
