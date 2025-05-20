import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithm."""

    # Define a fitness function to minimize route distance
    def fitness(route):
        distance = calculate_route_distance(route, _distances)
        return 1 / distance

    # Create a population of random routes
    population = [np.random.permutation(len(_distances)) for _ in range(100)]

    # Run the genetic algorithm for 100 generations
    for generation in range(100):
        # Select the fittest routes
        fittest_routes = sorted(population, key=fitness, reverse=True)[:50]

        # Create a new population by crossing over and mutating fittest routes
        new_population = []
        for i in range(0, len(fittest_routes), 2):
            parent1 = fittest_routes[i]
            parent2 = fittest_routes[i+1]
            crossover_point = np.random.randint(len(_distances))
            child1 = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))
            child2 = np.concatenate((parent2[:crossover_point], parent1[crossover_point:]))
            new_population.append(child1)
            new_population.append(child2)

        # Add mutated routes to the new population
        for route in fittest_routes:
            mutated_route = route.copy()
            np.random.shuffle(mutated_route)
            new_population.append(mutated_route)

        population = new_population

    # Return the best route found
    best_route = max(population, key=fitness)
    return best_route
