def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithm."""

    # Define the chromosome representation
    class Route(funsearch.Chromosome):
        def __init__(self, route):
            self.route = route

        def fitness(self):
            return -calculate_route_distance(self.route, _distances)

    # Create a population of routes
    population = [Route(route) for route in generate_random_routes(_distances)]

    # Run the genetic algorithm
    best_route = funsearch.genetic_algorithm(population, _distances)

    # Return the best route as a tuple
    return best_route.route
