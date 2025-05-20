def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2, incorporating genetic algorithms.

    This function uses a genetic algorithm to search for the optimal route.
    """

    # Define the chromosome representation
    class Route(funsearch.Chromosome):
        def __init__(self, route):
            self.route = route

        def fitness(self):
            return -calculate_route_distance(self.route, _distances)

    # Create the genetic algorithm instance
    ga = funsearch.GeneticAlgorithm(Route, _distances)

    # Run the genetic algorithm
    best_route = ga.run()

    # Return the best route
    return best_route.route
