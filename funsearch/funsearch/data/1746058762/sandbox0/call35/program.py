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

    # Use genetic algorithm to find a good solution
    population_size = 100
    num_generations = 100
    tournament_size = 3
    crossover_rate = 0.8
    mutation_rate = 0.2

    # Create the population
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Evaluate the population
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Iterate over generations
    for generation in range(num_generations):

        # Tournament selection
        selected_population = funsearch.tournament_selection(population, fitness_values, tournament_size)

        # Crossover
        offspring = funsearch.crossover(selected_population, crossover_rate)

        # Mutation
        mutated_offspring = funsearch.mutation(offspring, mutation_rate)

        # Evaluate the offspring
        offspring_fitness_values = [calculate_route_distance(route, _distances) for route in offspring]

        # Update the population
        population = funsearch.generational_update(population, offspring, fitness_values, offspring_fitness_values)

        # Get the best route
        best_route_index = np.argmin(fitness_values)
        best_route = population[best_route_index]

        # Print the best route for each generation
        print(f"Generation {generation}: {calculate_route_distance(best_route, _distances)}")

    # Return the best route
    return best_route

