def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a population of routes
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Evaluate the fitness of each route
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Iterate until the desired number of generations is reached
    for generation in range(num_generations):
        # Select the best routes for reproduction
        best_routes = np.argsort(fitness_values)[:population_size // 2]

        # Create new offspring through crossover and mutation
        new_population = []
        for i in range(population_size // 2):
            parent1 = population[best_routes[i]]
            parent2 = population[best_routes[np.random.randint(population_size // 2)]]

            # Perform crossover
            crossover_point = np.random.randint(len(_distances))
            child = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))

            # Perform mutation
            if np.random.rand() < mutation_rate:
                mutation_point1 = np.random.randint(len(_distances))
                mutation_point2 = np.random.randint(len(_distances))
                child[mutation_point1], child[mutation_point2] = child[mutation_point2], child[mutation_point1]

            new_population.append(child)

        # Evaluate the fitness of the new routes
        new_fitness_values = [calculate_route_distance(route, _distances) for route in new_population]

        # Update the population and fitness values
        population = new_population
        fitness_values = new_fitness_values

    # Return the route with the best fitness value
    best_route_index = np.argmin(fitness_values)
    return population[best_route_index]
