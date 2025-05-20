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

    # Use a genetic algorithm to find the best route.
    population_size = 100
    num_generations = 50
    crossover_rate = 0.8
    mutation_rate = 0.2

    # Create a population of random routes.
    population = np.random.permutation(len(_distances), size=(population_size, len(_distances)))

    # Evaluate the fitness of each route.
    fitness_values = np.array([calculate_route_distance(route, _distances) for route in population])

    # Run the genetic algorithm for the specified number of generations.
    for generation in range(num_generations):
        # Select parents based on their fitness values.
        parents = np.argsort(fitness_values)[:population_size // 2]

        # Create offspring by crossing over and mutating.
        offspring = []
        for i in range(population_size // 2):
            parent1 = population[parents[i]]
            parent2 = population[parents[population_size - i - 1]]

            # Perform single-point crossover.
            crossover_point = np.random.randint(1, len(_distances) - 1)
            offspring.append(np.concatenate((parent1[:crossover_point], parent2[crossover_point:])))

        # Mutate the offspring.
        for route in offspring:
            for i in range(len(_distances)):
                if np.random.rand() < mutation_rate:
                    j = np.random.randint(len(_distances))
                    route[i], route[j] = route[j], route[i]

        # Evaluate the fitness of the offspring.
        fitness_values_offspring = np.array([calculate_route_distance(route, _distances) for route in offspring])

        # Replace the least fit routes with the offspring.
        population[np.argsort(fitness_values)[-population_size // 2:]] = offspring
        fitness_values = np.min((fitness_values, fitness_values_offspring), axis=0)

    # Return the best route.
    best_route_index = np.argmin(fitness_values)
    return population[best_route_index]

