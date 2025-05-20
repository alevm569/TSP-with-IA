def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a genetic algorithm to find the best route.
    population_size = 100
    generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create the initial population.
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Run the genetic algorithm.
    for generation in range(generations):
        # Evaluate the population.
        fitness = [calculate_route_distance(route, _distances) for route in population]

        # Select the fittest routes.
        sorted_fitness = np.argsort(fitness)
        elite = population[sorted_fitness[:int(population_size * 0.2)]]

        # Create the next generation.
        offspring = []
        for i in range(population_size - len(elite)):
            parent1 = np.random.choice(elite)
            parent2 = np.random.choice(elite)

            # Crossover.
            if np.random.rand() < crossover_rate:
                crossover_point = np.random.randint(1, len(_distances) - 1)
                offspring.append(np.concatenate((parent1[:crossover_point], parent2[crossover_point:])))

            # Mutation.
            if np.random.rand() < mutation_rate:
                mutation_point1 = np.random.randint(0, len(_distances))
                mutation_point2 = np.random.randint(0, len(_distances))
                offspring.append(np.concatenate((parent1[:mutation_point1], parent2[mutation_point2], parent1[mutation_point1:mutation_point2], parent2[:mutation_point2])))

        # Add the elite routes to the next generation.
        population = elite + offspring

    # Return the best route found.
    best_fitness = np.min([calculate_route_distance(route, _distances) for route in population])
    best_route = population[np.argmin([calculate_route_distance(route, _distances) for route in population])]
    return best_route
