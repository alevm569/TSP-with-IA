def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Apply the genetic algorithm to find a good route
    population_size = 100
    generations = 100
    mutation_rate = 0.1
    crossover_rate = 0.8

    # Create a population of routes
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Evaluate the fitness of each route
    fitness = [calculate_route_distance(route, _distances) for route in population]

    # Iterate over generations
    for generation in range(generations):
        # Select the fittest routes
        best_routes = population[np.argsort(fitness)[:population_size//2]]

        # Create new offspring routes through crossover and mutation
        new_population = []
        for i in range(population_size):
            parent1 = best_routes[np.random.randint(population_size//2)]
            parent2 = best_routes[np.random.randint(population_size//2)]
            offspring = funsearch.crossover(parent1, parent2, crossover_rate)
            funsearch.mutate(offspring, mutation_rate)
            new_population.append(offspring)

        # Evaluate the fitness of the new routes
        new_fitness = [calculate_route_distance(route, _distances) for route in new_population]

        # Replace the worst routes with the new offspring routes
        population = new_population
        fitness = new_fitness

    # Return the best route
    return population[np.argmin(fitness)]
