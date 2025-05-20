def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm
    population_size = 100
    num_genes = len(_distances)
    mutation_rate = 0.1
    crossover_rate = 0.7

    # Create a genetic algorithm object
    ga = funsearch.GeneticAlgorithm(population_size, num_genes, mutation_rate, crossover_rate)

    # Set the fitness function
    ga.set_fitness_function(calculate_route_distance, _distances)

    # Run the genetic algorithm
    ga.run(max_generations=100)

    # Get the best route
    best_route = ga.best_individual()

    return best_route
