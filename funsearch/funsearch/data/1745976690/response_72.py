def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or combine multiple heuristics to improve performance.
    # Consider using a hybrid approach that combines different strategies.

    # Use a genetic algorithm to explore the search space.
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a genetic algorithm object.
    ga = funsearch.GeneticAlgorithm(
        population_size=population_size,
        num_generations=num_generations,
        crossover_rate=crossover_rate,
        mutation_rate=mutation_rate,
    )

    # Run the genetic algorithm to find the best route.
    best_route = ga.run(lambda: find_route(_distances))

    # Return the best route as a tuple of city indices.
    return best_route
