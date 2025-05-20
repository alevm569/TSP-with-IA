def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithm."""

    # Create a genetic algorithm instance
    ga = funsearch.GeneticAlgorithm(len(_distances))

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return -calculate_route_distance(route, _distances)

    # Run the genetic algorithm
    best_route = ga.run(fitness, generations=100, population_size=100)

    # Return the best route
    return best_route
