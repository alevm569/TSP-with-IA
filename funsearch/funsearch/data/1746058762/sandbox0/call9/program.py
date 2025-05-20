"""Implement a new version of the find_best_route function to solve the TSP problem.
AVOID brute force solutions, create new heuristics
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
    return int(distance)


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

    You may invent or combine heuristics from scratch, or use strategies such as:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design

    Routes must include all cities exactly once and return to the starting point.
    """
    """Improved version of `find_best_route_v1`."""

    # Use a genetic algorithm to generate candidate routes.
    population_size = 100
    generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a population of random routes.
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Evaluate the fitness of each route in the population.
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Iterate over generations.
    for generation in range(generations):
        # Select parents based on their fitness.
        parents = np.random.choice(population_size, size=population_size, replace=True, p=fitness_values / np.sum(fitness_values))

        # Create children by crossing over parents.
        children = []
        for i in range(population_size):
            parent1 = parents[i]
            parent2 = parents[(i + 1) % population_size]
            crossover_point = np.random.randint(1, len(_distances) - 1)
            child = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))

        # Mutate children by swapping two cities.
        for child in children:
            mutation_point1 = np.random.randint(0, len(_distances))
            mutation_point2 = np.random.randint(0, len(_distances))
            child[mutation_point1], child[mutation_point2] = child[mutation_point2], child[mutation_point1]

        # Evaluate the fitness of each child.
        child_fitness_values = [calculate_route_distance(route, _distances) for route in children]

        # Replace the worst routes in the population with the new children.
        worst_indices = np.argsort(fitness_values)[:population_size - len(children)]
        population[worst_indices] = children

        # Update the fitness values.
        fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Return the route with the lowest fitness value.
    best_route_index = np.argmin(fitness_values)
    return population[best_route_index]

