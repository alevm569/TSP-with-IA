def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Apply a genetic algorithm to find the best route
    population_size = 100
    num_generations = 100
    mutation_rate = 0.01
    crossover_rate = 0.8

    # Create a genetic algorithm object
    ga = funsearch.GeneticAlgorithm(population_size, num_generations, mutation_rate, crossover_rate)

    # Define the fitness function
    def fitness(route):
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm
    best_route = ga.run(fitness)

    return best_route
