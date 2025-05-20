def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to search for the best route.
    population_size = 100
    generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a population of routes.
    population = [funsearch.RandomTour().tour for _ in range(population_size)]

    # Evaluate the fitness of each route.
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Iterate over generations.
    for generation in range(generations):

        # Select parent routes for reproduction.
        parents = funsearch.TournamentSelection().select(population, fitness_values)

        # Create new offspring routes.
        offspring = funsearch.Crossover().crossover(parents, crossover_rate)

        # Mutate the offspring routes.
        funsearch.Mutation().mutate(offspring, mutation_rate)

        # Evaluate the fitness of the offspring routes.
        offspring_fitness_values = [calculate_route_distance(route, _distances) for route in offspring]

        # Replace the worst routes in the population with the offspring routes.
        population = funsearch.Replacement().replace(population, fitness_values, offspring, offspring_fitness_values)

        # Update the fitness values.
        fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Return the best route found.
    best_route_index = np.argmin(fitness_values)
    return population[best_route_index]
