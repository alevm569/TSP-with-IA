import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

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
    Improved version of find_best_route_v2, using a hybrid approach combining local search and ACO.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Generate an initial route using the nearest neighbor heuristic
    current_city = np.random.randint(len(matrix_distances))
    route = [current_city]

    # Perform ACO optimization to refine the route
    num_ants = 10
    num_iterations = 100

    for _ in range(num_iterations):
        # Generate new routes by simulating ant movements
        routes = []
        for _ in range(num_ants):
            route = []
            current_city = np.random.randint(len(matrix_distances))
            while len(route) < len(matrix_distances):
                # Select the next city based on ACO probabilities
                next_city = funsearch.ant_colony_step(route, matrix_distances)
                route.append(next_city)
            routes.append(route)

        # Evaluate the quality of each route and update pheromone levels
        funsearch.update_pheromones(routes, matrix_distances)

    # Use local search to further optimize the best route
    best_route = routes[np.argmin([calculate_route_distance(r, matrix_distances) for r in routes])]
    best_route = funsearch.local_search(best_route, matrix_distances)

    return best_route


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities.
    matrix_distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    float: The total distance of the route.
    """
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
