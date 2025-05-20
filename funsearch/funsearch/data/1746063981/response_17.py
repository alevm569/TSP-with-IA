def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route.
    population_size = 100
    mutation_rate = 0.01
    crossover_rate = 0.8
    num_generations = 1000

    # Create a population of routes.
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Evaluate the fitness of each route.
    fitness = [calculate_route_distance(route, _distances) for route in population]

    # Iterate over the generations.
    for generation in range(num_generations):

        # Select the fittest routes.
        sorted_indices = np.argsort(fitness)
        fittest_routes = population[sorted_indices[:int(population_size/2)]]

        # Create new routes through mutation and crossover.
        new_population = []
        for i in range(population_size):
            if np.random.rand() < mutation_rate:
                # Mutate a route by swapping two cities.
                index1, index2 = np.random.randint(0, len(_distances), 2)
                new_route = population[i][:index1] + population[i][index2] + population[i][index1:index2] + population[i][index2+1:]
            else:
                # Crossover two routes.
                parent1, parent2 = np.random.choice(fittest_routes, 2, replace=False)
                index = np.random.randint(0, len(_distances))
                new_route = parent1[:index] + parent2[index:]

            new_population.append(new_route)

        # Evaluate the fitness of the new routes.
        new_fitness = [calculate_route_distance(route, _distances) for route in new_population]

        # Replace the worst routes in the population with the new routes.
        population = new_population
        fitness = new_fitness

    # Return the best route.
    best_route_index = np.argmin(fitness)
    return population[best_route_index]
