import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid heuristic."""

    # Initialize population of routes
    population = np.random.permutation(len(_distances))

    # Run iterated local search to improve routes
    for _ in range(100):
        # Select two routes from population
        route1, route2 = np.random.choice(population, size=2, replace=False)

        # Perform 2-opt move between the two routes
        best_distance = math.inf
        for i in range(len(route1)):
            for j in range(len(route2)):
                new_distance = calculate_route_distance(route1[:i] + route2[j:] + route1[i:], _distances)
                if new_distance < best_distance:
                    best_distance = new_distance
                    best_route = route1[:i] + route2[j:] + route1[i:]

        # Replace worst route in population with the improved route
        population[np.argmax([calculate_route_distance(route, _distances) for route in population])] = best_route

    # Return the best route found
    return population[np.argmin([calculate_route_distance(route, _distances) for route in population])]


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
