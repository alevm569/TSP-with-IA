def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route
    population_size = 100
    generations = 100
    tournament_size = 3

    # Create a genetic algorithm object
    ga = funsearch.GeneticAlgorithm(
        population_size=population_size,
        generations=generations,
        tournament_size=tournament_size,
        mutation_rate=0.01,
        crossover_rate=0.8,
        distance_matrix=_distances,
    )

    # Run the genetic algorithm to find the best route
    best_route = ga.run()

    return best_route
