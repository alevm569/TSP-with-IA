def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a genetic algorithm to search for the best route.
    # Initialize the population of routes with random permutations of the cities.
    population = [np.random.permutation(len(_distances)) for _ in range(100)]

    # Define a fitness function to evaluate the total distance of a route.
    def fitness(route: np.ndarray) -> float:
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm until a stopping condition is met.
    while True:
        # Evaluate the fitness of each route in the population.
        fitnesses = [fitness(route) for route in population]

        # Select the best route in the population.
        best_route = population[np.argmin(fitnesses)]

        # Stop if the best route is less than or equal to 1000.
        if fitness(best_route) <= 1000:
            break

        # Create new routes by combining and mutating routes in the population.
        new_population = []
        for _ in range(100):
            parent1 = population[np.random.randint(len(population))]
            parent2 = population[np.random.randint(len(population))]

            # Combine the two parents to create a new route.
            crossover_point = np.random.randint(len(parent1))
            child = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))

            # Mutate the new route by swapping two cities.
            mutation_point1 = np.random.randint(len(child))
            mutation_point2 = np.random.randint(len(child))
            child[mutation_point1], child[mutation_point2] = child[mutation_point2], child[mutation_point1]

            new_population.append(child)

        population = new_population

    return best_route
