def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithm."""

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return -calculate_route_distance(route, _distances)

    # Create a population of routes
    population = [funsearch.random_permutation(len(_distances)) for _ in range(100)]

    # Run the genetic algorithm
    best_route = funsearch.genetic_algorithm(population, fitness, max_generations=1000)

    # Return the best route
    return best_route
