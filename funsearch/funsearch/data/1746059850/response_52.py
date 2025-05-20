def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using genetic algorithms.

    This function implements a genetic algorithm to find the optimal route.
    """

    # Define the number of cities
    num_cities = len(_distances)

    # Create a population of random routes
    population = [np.random.permutation(num_cities) for _ in range(100)]

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return -calculate_route_distance(route, _distances)

    # Run the genetic algorithm
    best_route = funsearch.genetic_algorithm(population, fitness, max_generations=1000)

    # Return the best route
    return best_route
