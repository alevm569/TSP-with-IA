def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route.
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.2

    # Create a population of routes.
    population = np.random.permutation(len(_distances), size=(population_size, len(_distances)))

    # Evaluate the fitness of each route.
    fitness = np.apply_along_axis(calculate_route_distance, 1, population, _distances)

    # Iterate over generations.
    for generation in range(num_generations):
        # Select the fittest routes for reproduction.
        parents = population[np.argsort(fitness)[-population_size//2:]]

        # Create the next generation of routes.
        offspring = []
        for i in range(population_size - len(parents)):
            # Crossover two parent routes.
            parent1 = np.random.choice(parents)
            parent2 = np.random.choice(parents)
            crossover_point = np.random.randint(1, len(_distances))
            offspring.append(np.concatenate((parent1[:crossover_point], parent2[crossover_point:])))

        # Mutate some routes.
        for route in offspring:
            if np.random.rand() < mutation_rate:
                index1 = np.random.randint(0, len(_distances))
                index2 = np.random.randint(0, len(_distances))
                route[index1], route[index2] = route[index2], route[index1]

        # Evaluate the fitness of the new routes.
        fitness = np.apply_along_axis(calculate_route_distance, 1, population, _distances)

        # Replace the least fittest routes with the new routes.
        population[np.argsort(fitness)[:population_size - len(parents)]] = offspring

    # Return the best route.
    return population[np.argmin(fitness)]
