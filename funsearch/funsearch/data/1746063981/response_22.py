def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use a genetic algorithm to find the best route
    population_size = 100
    num_generations = 100
    crossover_probability = 0.8
    mutation_probability = 0.2

    # Create a population of routes
    population = funsearch.Population(population_size, len(_distances))

    # Evaluate the fitness of each route in the population
    population.evaluate(evaluate)

    # Run the genetic algorithm for the specified number of generations
    for generation in range(num_generations):
        # Select parents for crossover
        parents = population.select()

        # Create a new population of offspring
        offspring = funsearch.crossover(parents, crossover_probability)

        # Apply mutation to the offspring
        funsearch.mutate(offspring, mutation_probability)

        # Evaluate the fitness of the offspring
        offspring.evaluate(evaluate)

        # Add the offspring to the population
        population.add_population(offspring)

    # Return the best route found
    best_route = population.best_route
    return best_route
