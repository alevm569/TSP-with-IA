def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithms."""

    # Define the chromosome structure
    class Route(funsearch.Chromosome):
        def distance(self) -> float:
            return calculate_route_distance(self.genes, _distances)

    # Create a genetic algorithm operator
    crossover = funsearch.crossover.SinglePointCrossover()
    mutation = funsearch.mutation.SwapMutation()

    # Initialize the population
    population = funsearch.population.Population(Route, size=100)

    # Run the genetic algorithm
    best_route = funsearch.ga.genetic_algorithm(population, crossover, mutation)

    # Return the best route
    return best_route.genes
