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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
    - Genetic Algorithm
    - Tabu Search

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize population
    population_size = 100
    population = np.random.permutation(len(matrix_distances)).reshape(population_size, -1)

    # Create a fitness function
    def fitness(route: np.ndarray) -> float:
        total_distance = 0
        for i in range(len(route)):
            total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
        return total_distance

    # Run the genetic algorithm
    best_route = funsearch.genetic_algorithm(population, fitness, tournament_size=3, mutation_rate=0.05, elitism=True)

    # Run the tabu search
    best_route = funsearch.tabu_search(best_route, fitness, max_iterations=1000)

    return best_route
