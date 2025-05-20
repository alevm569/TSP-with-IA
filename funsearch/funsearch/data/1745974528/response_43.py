def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route
    population_size = 100
    generations = 100
    mutation_rate = 0.1
    crossover_rate = 0.8

    # Create a genetic algorithm object
    ga = funsearch.GA(
        population_size=population_size,
        generations=generations,
        mutation_rate=mutation_rate,
        crossover_rate=crossover_rate,
        fitness_function=calculate_route_distance,
        distance_matrix=_distances,
    )

    # Run the genetic algorithm
    ga.run()

    # Return the best route found
    return ga.best_route
