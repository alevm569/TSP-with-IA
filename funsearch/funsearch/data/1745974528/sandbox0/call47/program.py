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
    # Use a genetic algorithm to find the best route
    population_size = 100
    num_generations = 100
    mutation_rate = 0.1
    crossover_rate = 0.8

    # Create a population of routes
    population = funsearch.generate_population(len(_distances), population_size)

    # Evaluate the fitness of each route
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Evolve the population for num_generations
    for generation in range(num_generations):
        # Selection
        parents = funsearch.selection(population, fitness_values)

        # Crossover
        offspring = funsearch.crossover(parents, crossover_rate)

        # Mutation
        funsearch.mutation(offspring, mutation_rate)

        # Evaluate the fitness of the offspring
        offspring_fitness_values = [calculate_route_distance(route, _distances) for route in offspring]

        # Replace the worst routes in the population with the offspring
        population = funsearch.replace_worst(population, fitness_values, offspring, offspring_fitness_values)

        # Update the fitness values
        fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Return the best route
    best_route_index = np.argmin(fitness_values)
    return population[best_route_index]


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

