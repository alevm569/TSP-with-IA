def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithms."""

    # Define a fitness function to evaluate the total distance of a route
    def fitness(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Initialize a population of random routes
    population = [random_route(len(_distances)) for _ in range(100)]

    # Run a genetic algorithm for 100 generations
    for generation in range(100):
        # Select the fittest routes for reproduction
        fittest_routes = sorted(population, key=fitness)[:50]

        # Create new routes through crossover and mutation
        new_population = []
        for i in range(0, len(population), 2):
            parent1, parent2 = fittest_routes[i], fittest_routes[i + 1]
            crossover_point = random.randint(1, len(parent1) - 1)
            child1 = parent1[:crossover_point] + parent2[crossover_point:]
            child2 = parent2[:crossover_point] + parent1[crossover_point:]
            new_population.extend([mutate(child1), mutate(child2)])

        population = new_population

    # Return the route with the lowest fitness (shortest total distance)
    best_route = sorted(population, key=fitness)[0]
    return best_route


# Helper functions for generating random routes, mutating routes, etc.
# ...
