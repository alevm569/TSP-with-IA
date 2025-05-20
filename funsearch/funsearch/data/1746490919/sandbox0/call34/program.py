"""Implement a new version of the find_best_route function to solve the TSP problem.
AVOID brute force solutions, create new heuristics and if you are going to use randomness, stabilize it by setting a seed to ensure reproducibility.
PROVIDE just the python code for the new version of the function, i.e. find_best_route_vx"""
import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



def calculate_route_distance(route: tuple[int, ...], distances: ndarray) -> float:
    """
    function to calculate the total distance of a given route.
    sum the distances between cities in the route
    """
    distance = sum(distances[route[i], route[i + 1]] for i in range(len(route) - 1))
    # add the distance from the last city to the first city
    distance += distances[route[-1], route[0]]
    return float(distance)


def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.
    
    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    You may use at least one strategy or combine two or more heuristics from the list below:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design
        - aco (ant colony optimization)
        - genetic algorithms
        - k-opt
        - tabu search

    Routes must include all cities exactly once and return to the starting point.
    """
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

