import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def calculate_route_distance(route: tuple[int], distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""
    # Use a genetic algorithm to find a good initial route
    num_cities = len(_distances)
    population_size = 100
    mutation_rate = 0.1
    crossover_rate = 0.5

    # Create a population of routes
    population = np.random.permutation(num_cities, size=(population_size, num_cities))

    # Evaluate the fitness of each route
    fitness = np.array([calculate_route_distance(route, _distances) for route in population])

    # Run the genetic algorithm until convergence
    for generation in range(100):
        # Select parents
        parent1 = population[np.argmin(fitness)]
        parent2 = population[np.random.randint(population_size)]

        # Crossover
        crossover_point = np.random.randint(num_cities)
        child = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))

        # Mutation
        if np.random.rand() < mutation_rate:
            mutation_point1 = np.random.randint(num_cities)
            mutation_point2 = np.random.randint(num_cities)
            child[mutation_point1], child[mutation_point2] = child[mutation_point2], child[mutation_point1]

        # Evaluate the fitness of the new route
        fitness[population == child] = calculate_route_distance(child, _distances)

        # Replace the worst route with the new route
        population[np.argmax(fitness)] = child

    # Return the best route found
    return population[np.argmin(fitness)]
