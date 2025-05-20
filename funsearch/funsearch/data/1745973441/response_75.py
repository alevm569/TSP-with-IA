def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route
    population_size = 100
    num_generations = 50
    tournament_size = 5
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a genetic algorithm object
    ga = funsearch.GeneticAlgorithm(
        population_size=population_size,
        num_generations=num_generations,
        tournament_size=tournament_size,
        crossover_rate=crossover_rate,
        mutation_rate=mutation_rate,
        fitness_function=calculate_route_distance,
        distance_matrix=_distances,
    )

    # Run the genetic algorithm
    best_route = ga.run()

    # Return the best route
    return best_route
