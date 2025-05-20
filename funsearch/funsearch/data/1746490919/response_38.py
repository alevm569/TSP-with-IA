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
    """
    Improved version of find_best_route_v2 using ACO heuristic.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    A permutation of cities that minimizes the total route distance.
    """

    # Initialize ACO parameters
    num_cities = len(matrix_distances)
    num_ants = 10
    num_iterations = 100
    alpha = 1  # pheromone strength
    beta = 2  # distance influence
    rho = 0.1  # pheromone evaporation rate

    # Initialize pheromone matrix
    pheromone_matrix = np.ones((num_cities, num_cities))

    # Initialize best route
    best_route = None
    best_distance = float('inf')

    # Run ACO algorithm
    for _ in range(num_iterations):
        # Send ants to explore routes
        routes = []
        for _ in range(num_ants):
            route = find_aco_route(matrix_distances, pheromone_matrix, alpha, beta)
            routes.append(route)

        # Update pheromone matrix
        pheromone_matrix *= (1 - rho)
        for route in routes:
            distance = calculate_route_distance(route, matrix_distances)
            for i in range(num_cities):
                for j in range(num_cities):
                    pheromone_matrix[route[i]][route[j]] += 1 / distance

        # Update best route
        for route in routes:
            distance = calculate_route_distance(route, matrix_distances)
            if distance < best_distance:
                best_distance = distance
                best_route = route

    return best_route

def find_aco_route(matrix_distances: np.ndarray, pheromone_matrix: np.ndarray, alpha: float, beta: float) -> tuple[int, ...]:
    """
    Finds a route using ACO heuristic.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)
    pheromone_matrix (np.ndarray): A matrix of pheromones, shape (n, n)
    alpha (float): Pheromone strength
    beta (float): Distance influence

    Returns:
    A route as a tuple of city indices.
    """

    num_cities = len(matrix_distances)
    route = np.zeros(num_cities, dtype=int)

    # Start from a random city
    current_city = np.random.randint(num_cities)
    route[0] = current_city

    # Generate remaining cities in the route
    for i in range(1, num_cities):
        # Calculate probabilities of moving to each city
        probabilities = pheromone_matrix[current_city] ** alpha * (1 / matrix_distances[current_city]) ** beta

        # Normalize probabilities
        probabilities /= np.sum(probabilities)

        # Choose next city based on probabilities
        next_city = np.random.choice(num_cities, p=probabilities)
        route[i] = next_city

        # Update current city
        current_city = next_city

    return route

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A route as a tuple of city indices
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    The total distance of the route.
    """

    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
