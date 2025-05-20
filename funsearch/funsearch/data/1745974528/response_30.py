def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route.
    population_size = 50
    mutation_rate = 0.1
    tournament_size = 3
    num_generations = 100

    # Create a population of routes.
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Evaluate the fitness of each route.
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Run the genetic algorithm.
    for generation in range(num_generations):
        # Select parents.
        parents = funsearch.tournament_selection(population, fitness_values, tournament_size)

        # Create offspring.
        offspring = funsearch.crossover(parents, _distances)

        # Mutate offspring.
        funsearch.mutate(offspring, mutation_rate)

        # Evaluate the fitness of the offspring.
        offspring_fitness_values = [calculate_route_distance(route, _distances) for route in offspring]

        # Replace the least fit routes with the offspring.
        worst_fitness_index = np.argmin(fitness_values)
        population[worst_fitness_index] = offspring[np.argmin(offspring_fitness_values)]
        fitness_values[worst_fitness_index] = offspring_fitness_values[np.argmin(offspring_fitness_values)]

    # Return the best route.
    best_route_index = np.argmin(fitness_values)
    return population[best_route_index]
