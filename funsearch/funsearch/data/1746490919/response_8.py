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


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO heuristic."""
    np.random.seed(42)  # Set a seed for reproducibility

    # Initialize ACO parameters
    num_cities = len(_distances)
    num_ants = 10
    alpha = 1.0  # Pheromone strength
    beta = 2.0  # Heuristic strength
    rho = 0.1  # Pheromone evaporation rate
    Q = 1  # Reward for finding a shorter route

    # Initialize pheromone matrix
    pheromones = np.ones((num_cities, num_cities))

    # Initialize best route and distance
    best_route = None
    best_distance = float('inf')

    # Run ACO algorithm for a fixed number of iterations
    for _ in range(100):
        # Send ants to explore the graph
        routes = []
        for _ in range(num_ants):
            route = ant_tour(_distances, pheromones, alpha, beta)
            routes.append(route)

        # Update pheromones based on the routes found by the ants
        for route in routes:
            distance = calculate_route_distance(route, _distances)
            if distance < best_distance:
                best_distance = distance
                best_route = route

            for i in range(num_cities):
                for j in range(num_cities):
                    pheromones[i][j] *= (1 - rho)  # Evaporation
                    if j in route[i:]:
                        pheromones[i][j] += Q / distance  # Deposition

    return best_route


# Helper function for ACO algorithm
def ant_tour(_distances: np.ndarray, pheromones: np.ndarray, alpha: float, beta: float) -> tuple[int, ...]:
    # Initialize ant's route
    route = []
    unvisited = set(range(len(_distances)))

    # Start from a random city
    current_city = np.random.choice(list(unvisited))
    route.append(current_city)
    unvisited.remove(current_city)

    # Explore the graph until all cities are visited
    while unvisited:
        # Choose the next city based on pheromone and heuristic information
        next_city = max(unvisited, key=lambda city: pheromones[current_city][city] ** alpha * (1 / _distances[current_city][city]) ** beta)
        route.append(next_city)
        unvisited.remove(next_city)
        current_city = next_city

    return route
