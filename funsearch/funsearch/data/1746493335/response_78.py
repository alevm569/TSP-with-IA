def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use genetic algorithm to find a potential solution
    population_size = 50
    num_generations = 100
    mutation_rate = 0.1
    crossover_rate = 0.8

    # Create a population of routes
    population = [np.random.permutation(np.arange(len(_distances))) for _ in range(population_size)]

    # Run genetic algorithm for num_generations
    for generation in range(num_generations):
        # Evaluate the fitness of each route
        fitness = [calculate_route_distance(route, _distances) for route in population]

        # Select parents for crossover
        parents = np.random.choice(population_size, size=population_size, p=fitness/np.sum(fitness))

        # Create new offspring through crossover
        offspring = []
        for i in range(population_size):
            parent1 = population[parents[2*i]]
            parent2 = population[parents[2*i+1]]
            crossover_point = np.random.randint(1, len(_distances) - 1)
            offspring.append(np.concatenate((parent1[:crossover_point], parent2[crossover_point:])))

        # Apply mutation to offspring
        for route in offspring:
            for i in range(len(route)):
                if np.random.rand() < mutation_rate:
                    j = np.random.randint(len(_distances))
                    route[i], route[j] = route[j], route[i]

        # Replace worst routes in population with offspring
        population = sorted(population, key=calculate_route_distance, reverse=False)[:population_size]

    # Return the best route found
    best_route = population[0]
    return best_route
