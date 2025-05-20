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
    Improved version of find_best_route_v2.

    Uses a combination of local search and genetic algorithms.

    Parameters:
        distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        A permutation of cities that minimizes the total route distance.
    """

    # Generate an initial solution using the nearest neighbor heuristic.
    current_city = np.random.randint(len(distances))
    route = [current_city]

    # Use genetic algorithms to improve the initial solution.
    population = funsearch.generate_population(len(distances))
    best_route = population[0]
    best_distance = calculate_route_distance(best_route, distances)

    # Run the genetic algorithm for a specified number of generations.
    for generation in range(100):
        population = funsearch.evolve_population(population, distances)
        for route in population:
            distance = calculate_route_distance(route, distances)
            if distance < best_distance:
                best_route = route
                best_distance = distance

    # Perform local search to further improve the best route.
    for i in range(100):
        best_route = funsearch.local_search(best_route, distances)

    return best_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
