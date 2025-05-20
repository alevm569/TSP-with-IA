def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Version of find_best_route using a genetic algorithm."""

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Initialize the genetic algorithm
    population_size = 100
    num_generations = 100
    crossover_probability = 0.8
    mutation_probability = 0.2

    # Create the genetic algorithm instance
    ga = funsearch.GeneticAlgorithm(fitness)

    # Run the genetic algorithm
    best_route = ga.optimize(population_size, num_generations, crossover_probability, mutation_probability)

    # Return the best route
    return best_route
