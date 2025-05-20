import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO."""

    num_cities = len(matrix_distances)

    # Initialize ACO parameters
    num_ants = 10
    alpha = 1.0  # pheromone weight
    beta = 2.0  # heuristic weight
    rho = 0.1  # pheromone evaporation rate

    # Initialize pheromone matrix
    pheromone_matrix = np.ones((num_cities, num_cities))

    # Initialize best route
    best_route = None
    best_distance = float('inf')

    for iteration in range(100):
        # Run ACO algorithm
        ant_routes = []
        for _ in range(num_ants):
            ant_route = find_ant_route(matrix_distances, pheromone_matrix)
            ant_routes.append(ant_route)

        # Update pheromone matrix
        for route in ant_routes:
            distance = calculate_route_distance(route, matrix_distances)
            for i in range(num_cities):
                for j in range(num_cities):
                    pheromone_matrix[i][j] *= (1 - rho)  # Evaporation
                    if route[i] == route[j]:
                        pheromone_matrix[i][j] += 1 / distance  # Deposition

        # Update best route
        for route in ant_routes:
            distance = calculate_route_distance(route, matrix_distances)
            if distance < best_distance:
                best_distance = distance
                best_route = route

    return best_route

def find_ant_route(matrix_distances: np.ndarray, pheromone_matrix: np.ndarray) -> tuple[int, ...]:
    """Find a route for an ant using ACO."""
    num_cities = len(matrix_distances)
    route = [0]  # Start from the first city

    while len(route) < num_cities:
        # Select the next city using the pheromone and heuristic probabilities
        probabilities = pheromone_matrix[route[-1]] ** alpha * (1 / matrix_distances[route[-1]]) ** beta
        probabilities /= np.sum(probabilities)
        next_city = np.random.choice(num_cities, p=probabilities)

        if next_city not in route:
            route.append(next_city)

    return route

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return distance
