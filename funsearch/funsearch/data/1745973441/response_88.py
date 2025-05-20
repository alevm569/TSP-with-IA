def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with genetic algorithm."""

    # Define the genetic algorithm parameters
    population_size = 100
    num_generations = 100
    mutation_rate = 0.01

    # Create a genetic algorithm object
    ga = funsearch.GeneticAlgorithm(population_size, num_generations, mutation_rate)

    # Define the fitness function
    def fitness(route: np.ndarray) -> float:
        return -calculate_route_distance(route, _distances)

    # Run the genetic algorithm
    best_route = ga.run(fitness, len(_distances))

    # Return the best route
    return best_route
