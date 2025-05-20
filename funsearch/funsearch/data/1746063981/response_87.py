def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithm."""

    # Create a Genetic Algorithm object
    ga = funsearch.GeneticAlgorithm(
        population_size=100,
        num_generations=100,
        crossover_rate=0.8,
        mutation_rate=0.2,
        fitness_function=calculate_route_distance,
        distance_matrix=_distances,
    )

    # Run the Genetic Algorithm
    best_route = ga.run()

    # Return the best route
    return best_route
