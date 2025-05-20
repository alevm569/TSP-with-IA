def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route.
    num_cities = len(_distances)
    population_size = 100
    generations = 100
    tournament_size = 3
    mutation_rate = 0.1

    # Create a population of random routes.
    population = [np.random.permutation(num_cities) for _ in range(population_size)]

    # Evaluate the fitness of each route.
    fitness = [calculate_route_distance(route, _distances) for route in population]

    # Iterate over generations.
    for generation in range(generations):
        # Select parents for reproduction.
        parents = funsearch.tournament_selection(population, fitness, tournament_size)

        # Create offspring.
        offspring = []
        for parent1, parent2 in zip(parents, parents[population_size // 2:]):
            offspring.extend(funsearch.crossover(parent1, parent2))

        # Mutate offspring.
        for route in offspring:
            if np.random.rand() < mutation_rate:
                funsearch.mutate(route)

        # Evaluate the fitness of offspring.
        offspring_fitness = [calculate_route_distance(route, _distances) for route in offspring]

        # Update population.
        population = funsearch.generational_update(population, fitness, offspring, offspring_fitness)
        fitness = [calculate_route_distance(route, _distances) for route in population]

    # Return the best route.
    best_route_index = np.argmin(fitness)
    return population[best_route_index]
