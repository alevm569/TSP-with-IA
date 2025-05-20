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
    """Improved version of `find_best_route_v1`."""

    # Use genetic algorithm to find a potential solution
    population_size = 50
    num_generations = 100
    mutation_rate = 0.1
    crossover_rate = 0.8

    # Create a population of routes
    population = [np.random.permutation(np.arange(len(_distances))) for _ in range(population_size)]

    # Run genetic algorithm for num_generations
    for generation in range(num_generations):
        # Evaluate the fitness of each route
        fitness = [calculate_route_distance(route, _distances) for route in population]

        # Select parents for crossover
        parents = np.random.choice(population_size, size=population_size, p=fitness/np.sum(fitness))

        # Create new offspring through crossover
        offspring = []
        for i in range(population_size):
            parent1 = population[parents[2*i]]
            parent2 = population[parents[2*i+1]]
            crossover_point = np.random.randint(1, len(_distances) - 1)
            offspring.append(np.concatenate((parent1[:crossover_point], parent2[crossover_point:])))

        # Apply mutation to offspring
        for route in offspring:
            for i in range(len(route)):
                if np.random.rand() < mutation_rate:
                    j = np.random.randint(len(_distances))
                    route[i], route[j] = route[j], route[i]

        # Replace worst routes in population with offspring
        population = sorted(population, key=calculate_route_distance, reverse=False)[:population_size]

    # Return the best route found
    best_route = population[0]
    return best_route

