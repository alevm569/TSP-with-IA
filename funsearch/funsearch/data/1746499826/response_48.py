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

    This function employs a hybrid approach combining:
        - Nearest neighbor heuristic for initialization
        - Cheapest insertion heuristic for route construction
        - Tabu search algorithm for local search

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize route using nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    visited = set([start_city])

    while len(visited) < len(distances):
        current_city = route[-1]
        nearest_city = np.argmin(distances[current_city][~np.isin(np.arange(len(distances)), visited)])
        route.append(nearest_city)
        visited.add(nearest_city)

    # Perform tabu search for local optimization
    tabu_tenure = 10  # Number of iterations to remember in tabu list
    best_route = route
    best_distance = calculate_route_distance(route, distances)

    for _ in range(1000):  # Number of tabu iterations
        neighbor_route = make_neighbor(route)
        neighbor_distance = calculate_route_distance(neighbor_route, distances)

        if neighbor_distance < best_distance and neighbor_route not in funsearch.tabu_list:
            route = neighbor_route
            best_distance = neighbor_distance
            funsearch.tabu_list.append(neighbor_route)

        if len(funsearch.tabu_list) > tabu_tenure:
            funsearch.tabu_list.pop(0)

    return route


def make_neighbor(route: list[int]) -> list[int]:
    """Generates a new route by swapping two random cities."""
    np.random.seed(42)  # For reproducibility
    i, j = np.random.randint(len(route), size=2)
    route[i], route[j] = route[j], route[i]
    return route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i+1) % len(route)]]
    return total_distance
