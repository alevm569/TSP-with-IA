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

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using ACO.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities representing the best route.
    """

    # Initialize ACO parameters
    num_cities = len(distances)
    num_ants = 10
    alpha = 1  # pheromone strength
    beta = 2  # heuristic strength
    rho = 0.1  # pheromone evaporation rate
    Q = 1  # pheromone deposit amount

    # Initialize pheromone matrix
    pheromones = np.ones((num_cities, num_cities))

    # Initialize best route
    best_route = None
    best_distance = float('inf')

    # Run ACO for multiple iterations
    for epoch in range(100):
        # Send ants to explore the graph
        routes = []
        for _ in range(num_ants):
            route = funsearch.aco.ant_colony_optimization(distances, pheromones, alpha, beta, rho, Q)
            routes.append(route)

        # Update pheromones based on route distances
        for route in routes:
            distance = calculate_route_distance(route, distances)
            for i in range(num_cities):
                for j in range(num_cities):
                    pheromones[i][j] *= (1 - rho)  # Evaporation
                    if route[i] == route[j]:
                        pheromones[i][j] += Q / distance  # Deposition

        # Update best route
        for route in routes:
            distance = calculate_route_distance(route, distances)
            if distance < best_distance:
                best_distance = distance
                best_route = route

    return best_route

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i+1) % len(route)]]
    return total_distance
