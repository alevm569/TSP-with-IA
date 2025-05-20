def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route
    population_size = 100
    num_generations = 100
    mutation_rate = 0.1
    crossover_rate = 0.8

    population = funsearch.genetic_algorithm(
        _distances, population_size, num_generations, mutation_rate, crossover_rate
    )

    # Return the best route found
    return population.best_solution.route
