def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a genetic algorithm to find the best route
    population_size = 100
    generations = 100
    mutation_rate = 0.1

    # Create a genetic algorithm object
    ga = funsearch.GeneticAlgorithm(population_size=population_size, generations=generations, mutation_rate=mutation_rate)

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm
    best_route = ga.run(fitness, range(len(_distances)))

    # Return the best route
    return best_route
