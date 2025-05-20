def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using genetic algorithm.

    Uses a genetic algorithm to search for the best route.
    """

    # Create a genetic algorithm search object
    ga = funsearch.GeneticAlgorithmSearch()

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm search
    best_route = ga.search(fitness, len(_distances))

    return best_route
