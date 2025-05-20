def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using genetic algorithms.

    This function uses a genetic algorithm to find an optimal or near-optimal solution to the TSP problem.
    """

    # Define the chromosome structure
    class Route(funsearch.Chromosome):
        def __init__(self, route: list[int]):
            self.route = route

        def fitness(self):
            return -calculate_route_distance(self.route, _distances)

        def crossover(self, other):
            return Route(crossover_routes(self.route, other.route))

        def mutate(self):
            mutate_route(self.route)

    # Create the genetic algorithm
    ga = funsearch.GeneticAlgorithm(
        population_size=100,
        chromosome_class=Route,
        fitness_function=Route.fitness,
        crossover_function=Route.crossover,
        mutation_function=Route.mutate,
        tournament_size=3,
        elitism=True,
    )

    # Run the genetic algorithm
    best_route = ga.run(num_generations=1000)

    # Return the best route
    return best_route.route
