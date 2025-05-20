def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Apply the genetic algorithm to find the best route
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a genetic algorithm object
    ga = funsearch.GeneticAlgorithm(
        population_size=population_size,
        num_generations=num_generations,
        crossover_rate=crossover_rate,
        mutation_rate=mutation_rate,
        fitness_function=calculate_route_distance,
        distance_matrix=_distances,
    )

    # Run the genetic algorithm to find the best route
    best_route = ga.run()

    return best_route
