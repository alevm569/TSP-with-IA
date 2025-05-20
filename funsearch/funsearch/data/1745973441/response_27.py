def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a population of routes
    population = np.random.randint(low=0, high=_distances.shape[0], size=(population_size, _distances.shape[0]))

    # Evaluate the fitness of each route
    fitness = np.array([calculate_route_distance(route, _distances) for route in population])

    # Run the genetic algorithm
    for generation in range(num_generations):
        # Select parents
        parents = np.random.choice(population_size, size=population_size, p=fitness / np.sum(fitness))

        # Create children
        children = []
        for i in range(population_size):
            parent1 = population[parents[2 * i]]
            parent2 = population[parents[2 * i + 1]]
            child = crossover(parent1, parent2, crossover_rate)
            mutate(child, mutation_rate)
            children.append(child)

        # Evaluate the fitness of the children
        fitness_children = np.array([calculate_route_distance(route, _distances) for route in children])

        # Replace the worst routes in the population with the best children
        population[fitness > np.max(fitness_children)] = children[fitness_children < np.min(fitness)]
        fitness = np.array([calculate_route_distance(route, _distances) for route in population])

    # Return the route with the best fitness
    best_route_index = np.argmin(fitness)
    return population[best_route_index]
