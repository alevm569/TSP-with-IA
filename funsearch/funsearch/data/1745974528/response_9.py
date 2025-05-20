def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using genetic algorithms."""

    # Create a population of routes
    population = funsearch.Population(len(_distances))

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm
    best_route = funsearch.genetic_algorithm(population, fitness)

    return best_route
