def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Use a genetic algorithm to explore different route permutations.
    population_size = 100
    num_generations = 50
    mutation_rate = 0.1
    crossover_rate = 0.8

    # Initialize the population of routes.
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Run the genetic algorithm.
    for generation in range(num_generations):
        # Evaluate the fitness of each route.
        fitness = [calculate_route_distance(route, _distances) for route in population]

        # Select the fittest routes for reproduction.
        parents = np.random.choice(population_size, size=population_size, p=fitness / np.sum(fitness))

        # Create new routes through crossover and mutation.
        offspring = []
        for i in range(population_size):
            parent1 = parents[np.random.randint(population_size)]
            parent2 = parents[np.random.randint(population_size)]

            # Crossover operation.
            crossover_point = np.random.randint(1, len(_distances))
            offspring.append(np.concatenate((parent1[:crossover_point], parent2[crossover_point:])))

            # Mutation operation.
            if np.random.rand() < mutation_rate:
                mutation_index = np.random.randint(len(_distances))
                offspring.append(np.delete(np.append(population[i][:mutation_index], population[i][mutation_index + 1:]), mutation_index))

        # Replace the least fittest routes with the new offspring.
        population = np.array(sorted(population + offspring, key=calculate_route_distance)[:population_size])

    # Return the best route found.
    return population[np.argmin(fitness)]
