import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a hybrid approach that combines local search with a different heuristic.

    # Use a metaheuristic optimization algorithm, such as simulated annealing or genetic algorithm.
    # These algorithms can help find optimal or near-optimal solutions to the TSP problem.

    # Use a genetic algorithm to optimize the route.
    num_cities = len(_distances)
    population_size = 100
    mutation_rate = 0.1
    crossover_rate = 0.8

    def fitness(route):
        total_distance = 0
        for i in range(num_cities):
            total_distance += _distances[route[i]][route[(i + 1) % num_cities]]
        return total_distance

    # Create an initial population of routes.
    population = np.random.permutation(num_cities, size=(population_size, num_cities))

    # Run the genetic algorithm for a specified number of generations.
    for generation in range(100):
        # Select the fittest routes.
        sorted_routes = sorted(population, key=fitness)
        fittest_routes = sorted_routes[:population_size // 2]

        # Create the next generation of routes.
        new_population = []
        for i in range(population_size):
            # Crossover two parent routes.
            parent1 = np.random.choice(fittest_routes)
            parent2 = np.random.choice(fittest_routes)
            crossover_point = np.random.randint(1, num_cities)
            child = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))

            # Mutate the route.
            if np.random.rand() < mutation_rate:
                mutation_point1 = np.random.randint(0, num_cities)
                mutation_point2 = np.random.randint(0, num_cities)
                child[mutation_point1], child[mutation_point2] = child[mutation_point2], child[mutation_point1]

            new_population.append(child)

        population = new_population

    # Return the best route.
    best_route = sorted(population, key=fitness)[0]
    return best_route
