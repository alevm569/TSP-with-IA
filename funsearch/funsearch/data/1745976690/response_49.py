def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithm."""

    # Create a genetic algorithm search object
    ga = funsearch.GA(
        population_size=100,
        mutation_rate=0.1,
        crossover_rate=0.7,
        tournament_size=3,
        num_generations=100,
        distance_matrix=_distances,
    )

    # Run the genetic algorithm
    best_route = ga.run()

    return best_route
