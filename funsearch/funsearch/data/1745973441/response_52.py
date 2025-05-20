def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with genetic algorithm."""

    # Create a population of routes
    population = [np.random.permutation(len(_distances)) for _ in range(100)]

    # Evaluate the fitness of each route
    fitness = [calculate_route_distance(route, _distances) for route in population]

    # Perform genetic algorithm
    for generation in range(100):
        # Selection
        parents = np.random.choice(population, size=50, replace=False, p=fitness / np.sum(fitness))

        # Crossover
        children = []
        for i in range(0, len(parents), 2):
            parent1 = parents[i]
            parent2 = parents[i + 1]
            crossover_point = np.random.randint(1, len(_distances))
            child = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))

            # Mutation
            mutation_rate = 0.01
            for i in range(len(child)):
                if np.random.rand() < mutation_rate:
                    j = np.random.randint(0, len(_distances))
                    child[i], child[j] = child[j], child[i]

            children.append(child)

        # Evaluate the fitness of each child route
        child_fitness = [calculate_route_distance(route, _distances) for route in children]

        # Add the new children to the population
        population.extend(children)
        fitness.extend(child_fitness)

    # Return the route with the lowest fitness
    best_route_index = np.argmin(fitness)
    return population[best_route_index]
