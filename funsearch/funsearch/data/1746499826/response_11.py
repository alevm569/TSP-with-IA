import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using ACO."""

    # Initialize ACO parameters
    num_cities = len(_distances)
    num_ants = 10
    num_iterations = 100
    alpha = 1  # Influence of pheromone trails
    beta = 2  # Influence of heuristic information
    rho = 0.1  # Evaporation rate of pheromone trails

    # Initialize pheromone trails
    pheromone_trails = np.ones((num_cities, num_cities))

    # Run ACO algorithm
    for iteration in range(num_iterations):
        # Send ants to explore the graph
        for ant in range(num_ants):
            current_city = np.random.randint(num_cities)
            visited_cities = [current_city]

            while len(visited_cities) < num_cities:
                # Calculate probabilities of visiting each unvisited city
                probabilities = pheromone_trails[current_city] ** alpha * (1 / _distances[current_city]) ** beta
                probabilities /= np.sum(probabilities)

                # Choose the next city based on probabilities
                next_city = np.random.choice(num_cities, p=probabilities)
                visited_cities.append(next_city)
                current_city = next_city

            # Update pheromone trails
            for i in range(num_cities):
                for j in range(num_cities):
                    if i in visited_cities and j in visited_cities:
                        pheromone_trails[i][j] += 1
                    else:
                        pheromone_trails[i][j] *= (1 - rho)

    # Return the best route found by the ACO algorithm
    best_route = visited_cities
    return best_route
