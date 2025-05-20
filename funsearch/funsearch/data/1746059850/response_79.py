def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route
    population_size = 100
    num_generations = 100

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Create the genetic algorithm object
    ga = funsearch.GA(population_size=population_size, fitness_function=fitness)

    # Run the genetic algorithm
    ga.run(num_generations=num_generations)

    # Return the best route found
    return ga.best_individual.solution
