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

    # Use a genetic algorithm to search for the best route.
    population_size = 100
    num_generations = 100
    mutation_rate = 0.1
    crossover_rate = 0.8

    # Create the initial population of routes.
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Evaluate the fitness of each route in the population.
    fitness = [calculate_route_distance(route, _distances) for route in population]

    # Iterate over the generations.
    for generation in range(num_generations):
        # Select the fittest routes for reproduction.
        parents = funsearch.tournament_selection(population, fitness, k=2)

        # Create the next generation of routes.
        offspring = funsearch.crossover(parents, _distances, rate=crossover_rate)
        offspring = funsearch.mutation(offspring, _distances, rate=mutation_rate)

        # Evaluate the fitness of the offspring routes.
        fitness_offspring = [calculate_route_distance(route, _distances) for route in offspring]

        # Replace the least fittest routes in the population with the offspring routes.
        population = funsearch.elitism(population, fitness, offspring, fitness_offspring, k=population_size - len(parents))

        # Update the fitness of the population.
        fitness = [calculate_route_distance(route, _distances) for route in population]

    # Return the route with the lowest fitness.
    best_route_index = np.argmin(fitness)
    return population[best_route_index]

