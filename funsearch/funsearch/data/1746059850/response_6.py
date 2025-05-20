def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a genetic algorithm to search for the best route.
    population_size = 100
    num_generations = 100
    mutation_rate = 0.1
    crossover_rate = 0.8

    # Create the initial population of routes.
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Evaluate the fitness of each route in the population.
    fitness = [calculate_route_distance(route, _distances) for route in population]

    # Iterate over the generations.
    for generation in range(num_generations):
        # Select the fittest routes for reproduction.
        parents = funsearch.tournament_selection(population, fitness, k=2)

        # Create the next generation of routes.
        offspring = funsearch.crossover(parents, _distances, rate=crossover_rate)
        offspring = funsearch.mutation(offspring, _distances, rate=mutation_rate)

        # Evaluate the fitness of the offspring routes.
        fitness_offspring = [calculate_route_distance(route, _distances) for route in offspring]

        # Replace the least fittest routes in the population with the offspring routes.
        population = funsearch.elitism(population, fitness, offspring, fitness_offspring, k=population_size - len(parents))

        # Update the fitness of the population.
        fitness = [calculate_route_distance(route, _distances) for route in population]

    # Return the route with the lowest fitness.
    best_route_index = np.argmin(fitness)
    return population[best_route_index]
