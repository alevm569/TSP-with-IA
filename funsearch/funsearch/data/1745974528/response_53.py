def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithms."""
    num_cities = len(_distances)
    population_size = 100
    generations = 100

    # Initialize population
    population = np.random.permutation(num_cities, size=(population_size, num_cities))

    # Fitness function
    def fitness(route: tuple[int, ...]) -> float:
        distance = 0
        for i in range(num_cities):
            distance += _distances[route[i]][route[(i + 1) % num_cities]]
        return distance

    # Genetic operators
    def crossover(parent1: tuple[int, ...], parent2: tuple[int, ...]) -> tuple[int, ...]:
        mask = np.random.randint(2, size=num_cities)
        child = np.where(mask == 0, parent1, parent2)
        return child

    def mutation(route: tuple[int, ...]) -> tuple[int, ...]:
        i, j = np.random.randint(num_cities, size=2)
        route[i], route[j] = route[j], route[i]
        return route

    # Genetic algorithm
    for generation in range(generations):
        # Evaluate population
        fitness_values = np.array([fitness(route) for route in population])

        # Select parents
        parents = population[np.argsort(fitness_values)[:population_size // 2]]

        # Create new population
        population = []
        for _ in range(population_size):
            parent1, parent2 = np.random.choice(parents, size=2)
            child = crossover(parent1, parent2)
            child = mutation(child)
            population.append(child)

    # Return best route
    best_route = population[np.argmin([fitness(route) for route in population])]
    return best_route
