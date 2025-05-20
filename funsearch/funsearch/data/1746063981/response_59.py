def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithm."""

    # Initialize population of random routes
    population = [np.random.permutation(len(_distances)) for _ in range(50)]

    # Define fitness function
    def fitness(route: np.ndarray) -> float:
        return calculate_route_distance(route, _distances)

    # Run genetic algorithm
    best_route = funsearch.genetic_algorithm(population, fitness, max_generations=100)

    return best_route
