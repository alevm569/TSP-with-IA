import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v0(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Use a hybrid heuristic that combines nearest neighbor and local search
    # Generate an initial route using nearest neighbor
    start_city = np.random.randint(len(_distances))
    current_city = start_city
    route = [current_city]

    while len(route) < len(_distances):
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Perform local search to improve the route
    for _ in range(100):
        i, j = np.random.randint(len(route), size=2)
        route[i], route[j] = route[j], route[i]

    # Ensure that the route includes all cities exactly once and returns to the starting point
    if route[0] != start_city:
        route.append(start_city)

    return tuple(route)

def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Use a genetic algorithm to find the best route
    population_size = 100
    num_generations = 100
    mutation_rate = 0.1
    crossover_rate = 0.5

    # Create a population of routes
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Evaluate the fitness of each route
    fitness = [calculate_route_distance(route, _distances) for route in population]

    # Run the genetic algorithm
    for generation in range(num_generations):
        # Select parents
        parents = np.random.choice(population_size, size=population_size, p=fitness / np.sum(fitness))

        # Create children
        children = []
        for i in range(population_size):
            parent1 = parents[np.random.randint(population_size)]
            parent2 = parents[np.random.randint(population_size)]

            # Crossover
            crossover_point = np.random.randint(len(_distances))
            child = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))

            # Mutation
            if np.random.rand() < mutation_rate:
                mutation_point1 = np.random.randint(len(_distances))
                mutation_point2 = np.random.randint(len(_distances))
                child[mutation_point1], child[mutation_point2] = child[mutation_point2], child[mutation_point1]

            children.append(child)

        # Evaluate the fitness of the children
        fitness_children = [calculate_route_distance(route, _distances) for route in children]

        # Replace the worst parents with the best children
        worst_indices = np.argsort(fitness)[:population_size // 2]
        population[worst_indices] = children[worst_indices]
        fitness[worst_indices] = fitness_children[worst_indices]

    # Return the best route found
    best_route_index = np.argmin(fitness)
    return population[best_route_index]
