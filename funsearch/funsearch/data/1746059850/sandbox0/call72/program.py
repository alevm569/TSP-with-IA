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
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route.
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.2

    # Create a population of routes.
    population = np.random.permutation(len(_distances), size=(population_size, len(_distances)))

    # Evaluate the fitness of each route.
    fitness = np.apply_along_axis(calculate_route_distance, 1, population, _distances)

    # Iterate over generations.
    for generation in range(num_generations):
        # Select the fittest routes for reproduction.
        parents = population[np.argsort(fitness)[-population_size//2:]]

        # Create the next generation of routes.
        offspring = []
        for i in range(population_size - len(parents)):
            # Crossover two parent routes.
            parent1 = np.random.choice(parents)
            parent2 = np.random.choice(parents)
            crossover_point = np.random.randint(1, len(_distances))
            offspring.append(np.concatenate((parent1[:crossover_point], parent2[crossover_point:])))

        # Mutate some routes.
        for route in offspring:
            if np.random.rand() < mutation_rate:
                index1 = np.random.randint(0, len(_distances))
                index2 = np.random.randint(0, len(_distances))
                route[index1], route[index2] = route[index2], route[index1]

        # Evaluate the fitness of the new routes.
        fitness = np.apply_along_axis(calculate_route_distance, 1, population, _distances)

        # Replace the least fittest routes with the new routes.
        population[np.argsort(fitness)[:population_size - len(parents)]] = offspring

    # Return the best route.
    return population[np.argmin(fitness)]

