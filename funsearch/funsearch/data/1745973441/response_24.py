def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a genetic algorithm to find the best route.
    population_size = 100
    num_generations = 100
    mutation_rate = 0.1
    crossover_rate = 0.8

    # Create a genetic algorithm optimizer.
    optimizer = funsearch.GeneticOptimizer(
        population_size=population_size,
        num_generations=num_generations,
        mutation_rate=mutation_rate,
        crossover_rate=crossover_rate,
    )

    # Define the fitness function.
    def fitness_function(route):
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm.
    best_route = optimizer.optimize(fitness_function)

    # Return the best route as a tuple of city indices.
    return best_route
