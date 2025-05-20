def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Initialize population with random permutations of cities
    population = [np.random.permutation(len(_distances)) for _ in range(100)]

    # Define fitness function to minimize route distance
    def fitness(route: np.ndarray) -> float:
        return calculate_route_distance(route, _distances)

    # Perform genetic algorithm with mutation, crossover, and selection operators
    best_route = funsearch.genetic_algorithm(population, fitness, max_generations=1000)

    # Return the best route found
    return best_route
