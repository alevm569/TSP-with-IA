def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2`.

    Uses a genetic algorithm to find an optimal route.
    """

    # Define the chromosome structure
    class Route(funsearch.Chromosome):
        def fitness(self) -> float:
            return calculate_route_distance(self.genes, _distances)

        def crossover(self, other) -> tuple['Route', 'Route']:
            # Perform single-point crossover
            crossover_point = np.random.randint(1, len(self.genes))
            child1 = Route(self.genes[:crossover_point] + other.genes[crossover_point:])
            child2 = Route(other.genes[:crossover_point] + self.genes[crossover_point:])
            return child1, child2

        def mutate(self) -> None:
            # Perform inversion mutation
            mutation_point1, mutation_point2 = np.random.randint(0, len(self.genes), size=2)
            self.genes[mutation_point1], self.genes[mutation_point2] = self.genes[mutation_point2], self.genes[mutation_point1]

    # Create a population of routes
    population = [Route(np.random.permutation(len(_distances))) for _ in range(100)]

    # Run the genetic algorithm
    best_route = funsearch.genetic_algorithm(population, max_generations=1000)

    # Return the best route
    return best_route.genes
