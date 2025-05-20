def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using metaheuristics."""

    # Initialize population of candidate routes
    population = [np.random.permutation(len(_distances)) for _ in range(50)]

    # Define a fitness function to evaluate the total distance of a route
    def fitness(route):
        return calculate_route_distance(route, _distances)

    # Run a genetic algorithm to find the best route
    best_route = funsearch.genetic(population, fitness, max_generations=100)

    return best_route
