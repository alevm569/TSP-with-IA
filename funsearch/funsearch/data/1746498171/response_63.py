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
    Improved version of `find_best_route_v2` using ACO.
    """
    # Initialize ACO parameters
    num_cities = len(matrix_distances)
    num_ants = 10
    alpha = 1  # pheromone strength
    beta = 2  # distance preference
    evaporation_rate = 0.5  # pheromone evaporation rate

    # Initialize pheromone matrix
    pheromone_matrix = np.ones((num_cities, num_cities))

    # Initialize best route and distance
    best_route = None
    best_distance = float('inf')

    # Run ACO for multiple iterations
    for iteration in range(100):
        # Generate ant routes
        ant_routes = []
        for ant in range(num_ants):
            route = ACO(matrix_distances, pheromone_matrix, alpha, beta).find_route()
            ant_routes.append(route)

        # Update pheromone matrix
        pheromone_matrix *= (1 - evaporation_rate)
        for route in ant_routes:
            distance = calculate_route_distance(route, matrix_distances)
            for i in range(num_cities):
                for j in range(i + 1, num_cities):
                    pheromone_matrix[route[i]][route[j]] += 1 / distance

        # Update best route and distance
        for route in ant_routes:
            distance = calculate_route_distance(route, matrix_distances)
            if distance < best_distance:
                best_distance = distance
                best_route = route

    return best_route


class ACO:
    def __init__(self, matrix_distances, pheromone_matrix, alpha, beta):
        self.matrix_distances = matrix_distances
        self.pheromone_matrix = pheromone_matrix
        self.alpha = alpha
        self.beta = beta

    def find_route(self):
        num_cities = len(self.matrix_distances)
        np.random.seed(42)  # Set seed for reproducibility
        route = np.random.permutation(num_cities)

        while not self.is_valid_route(route):
            route = np.random.permutation(num_cities)

        return route

    def is_valid_route(self, route):
        return len(set(route)) == len(route) and route[0] == route[-1]

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.
    """
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return distance
