def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithm."""

    # Define the population size and number of generations
    population_size = 100
    num_generations = 100

    # Create the initial population of routes
    population = [funsearch.RandomPermutation(len(_distances)) for _ in range(population_size)]

    # Define the fitness function
    def fitness(route: funsearch.Permutation) -> float:
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm
    best_route = funsearch.genetic_algorithm(population, fitness, num_generations)

    # Return the best route
    return best_route.permutation
