def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use a genetic algorithm to find the best route.
    # The genetic algorithm will use a fitness function to determine the quality of each route.
    # The fittest routes will be selected to reproduce, and new routes will be created by combining the best routes.
    # The process will continue until a good route is found.

    # Create a fitness function.
    def fitness_function(route: tuple[int, ...]) -> float:
        """Calculates the total distance of a route."""
        total_distance = 0
        for i in range(len(route)):
            total_distance += _distances[route[i]][route[(i + 1) % len(route)]]
        return total_distance

    # Create a genetic algorithm.
    ga = funsearch.GeneticAlgorithm(fitness_function)

    # Run the genetic algorithm until a good route is found.
    best_route = ga.run()

    # Return the best route.
    return best_route
