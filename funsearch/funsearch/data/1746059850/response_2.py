def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using genetic algorithms."""

    # Define the chromosome structure
    class Route(funsearch.Chromosome):
        def __init__(self, route: list):
            super().__init__(route)

        def fitness(self):
            return -calculate_route_distance(self.route, _distances)

    # Create the population
    population = [Route(np.random.permutation(len(_distances))) for _ in range(100)]

    # Run the genetic algorithm
    best_route = funsearch.genetic_algorithm(population, 1000)

    # Return the best route
    return best_route.route
