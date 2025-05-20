def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use genetic algorithms to search for the best route.
    # This approach combines the strengths of genetic algorithms with the ability to represent complex solutions.

    # Define the fitness function to evaluate the total distance of a route.
    def fitness(route: list) -> float:
        total_distance = 0
        for i in range(len(route)):
            total_distance += _distances[route[i]][route[(i + 1) % len(route)]]
        return total_distance

    # Create a population of random routes.
    population = [np.random.permutation(len(_distances)) for _ in range(100)]

    # Run the genetic algorithm for a specified number of generations.
    for generation in range(100):
        # Select the fittest routes.
        fittest_routes = sorted(population, key=fitness)[:10]

        # Create new routes by crossing over and mutating.
        new_population = []
        for i in range(0, len(population), 2):
            parent1 = fittest_routes[np.random.randint(10)]
            parent2 = fittest_routes[np.random.randint(10)]
            crossover_point = np.random.randint(1, len(_distances))
            child1 = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))
            child2 = np.concatenate((parent2[:crossover_point], parent1[crossover_point:]))
            new_population.append(child1)
            new_population.append(child2)

        # Introduce mutations.
        for route in new_population:
            mutation_point1 = np.random.randint(0, len(_distances))
            mutation_point2 = np.random.randint(0, len(_distances))
            route[mutation_point1], route[mutation_point2] = route[mutation_point2], route[mutation_point1]

        population = new_population

    # Return the best route found.
    best_route = fittest_routes[0]
    return best_route
