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


def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use ACO (Ant Colony Optimization)
    num_ants = 10
    num_iterations = 100
    alpha = 1  # pheromone strength
    beta = 5  # distance preference

    # Initialize pheromone matrix
    pheromone_matrix = np.ones((len(_distances), len(_distances)))

    # Create a list of cities
    cities = list(range(len(_distances)))

    # Initialize best route
    best_route = None
    best_distance = float('inf')

    # Run ACO for the specified number of iterations
    for i in range(num_iterations):
        # Create ants
        ants = [Ant(_distances, pheromone_matrix, alpha, beta) for _ in range(num_ants)]

        # Send ants on their routes
        for ant in ants:
            ant.run()

        # Update pheromone matrix
        for ant in ants:
            for i in range(len(ant.route)):
                pheromone_matrix[ant.route[i]][ant.route[(i + 1) % len(ant.route)]] += 1 / ant.distance

        # Update best route
        for ant in ants:
            if ant.distance < best_distance:
                best_distance = ant.distance
                best_route = ant.route

    return best_route


class Ant:
    def __init__(self, distances, pheromone_matrix, alpha, beta):
        self.distances = distances
        self.pheromone_matrix = pheromone_matrix
        self.alpha = alpha
        self.beta = beta
        self.route = None
        self.distance = None

    def run(self):
        # Initialize route and distance
        self.route = [0]
        self.distance = 0

        # Visit remaining cities
        while len(self.route) < len(self.distances):
            # Get list of unvisited cities
            unvisited_cities = [i for i in range(len(self.distances)) if i not in self.route]

            # Calculate probabilities of visiting each unvisited city
            probabilities = []
            for city in unvisited_cities:
                probability = self.pheromone_matrix[self.route[-1]][city] ** self.alpha * (1 / self.distances[self.route[-1]][city]) ** self.beta
                probabilities.append(probability)

            # Select city with highest probability
            next_city = np.random.choice(unvisited_cities, p=probabilities / sum(probabilities))
            self.route.append(next_city)
            self.distance += self.distances[self.route[-2]][self.route[-1]]

    def __lt__(self, other):
        return self.distance < other.distance
