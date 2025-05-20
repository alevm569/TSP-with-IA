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
    """Improved version of `find_best_route_v2` with simulated annealing."""

    # Generate a random initial route
    route = np.random.permutation(len(_distances))

    # Perform simulated annealing
    temperature = 1000
    for _ in range(1000):
        # Randomly swap two cities in the route
        i, j = np.random.randint(0, len(_distances), 2)
        route[i], route[j] = route[j], route[i]

        # Calculate the change in distance
        distance_change = calculate_route_distance(route, _distances) - calculate_route_distance(route, _distances)

        # Accept the new route with probability based on temperature
        if distance_change < 0 or np.random.rand() < np.exp(-distance_change / temperature):
            pass
        else:
            # If not, reverse the swap
            route[i], route[j] = route[j], route[i]

        # Cool down the temperature
        temperature *= 0.99

    return route


def find_best_route_v4(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v3` with genetic algorithm."""

    # Create a genetic algorithm search object
    ga = funsearch.GeneticAlgorithm(
        population_size=100,
        num_generations=100,
        mutation_rate=0.05,
        crossover_rate=0.8,
        fitness_function=calculate_route_distance,
        distance_matrix=_distances,
    )

    # Run the genetic algorithm
    best_route = ga.search()

    return best_route


def find_best_route_v5(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v4` with ant colony optimization."""

    # Create an ant colony optimization object
    aco = funsearch.AntColonyOptimization(
        num_ants=100,
        num_iterations=100,
        pheromone_evaporation_rate=0.5,
        pheromone_deposit_factor=1.0,
        fitness_function=calculate_route_distance,
        distance_matrix=_distances,
    )

    # Run the ant colony optimization algorithm
    best_route = aco.search()

    return best_route
