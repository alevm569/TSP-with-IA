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

    # Initialize variables
    num_cities = len(_distances)
    best_route = None
    best_distance = float('inf')

    # Use a genetic algorithm
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.2

    # Create a population of routes
    population = [generate_random_route(num_cities) for _ in range(population_size)]

    # Evolve the population
    for generation in range(num_generations):
        # Evaluate the fitness of each route
        fitness_values = [calculate_route_distance(route, _distances) for route in population]

        # Select parents for crossover
        parents = tournament_selection(population, fitness_values)

        # Create a new population of offspring
        offspring = []
        for i in range(population_size):
            parent1, parent2 = parents[i]
            if np.random.rand() < crossover_rate:
                offspring.append(crossover(parent1, parent2))
            else:
                offspring.append(mutation(parent1))

        # Replace the population with the new offspring
        population = offspring

        # Find the best route in the current population
        best_route_in_population = population[np.argmin(fitness_values)]
        best_distance_in_population = fitness_values[np.argmin(fitness_values)]

        # Update the best route if necessary
        if best_distance_in_population < best_distance:
            best_distance = best_distance_in_population
            best_route = best_route_in_population

    # Return the best route found
    return best_route

