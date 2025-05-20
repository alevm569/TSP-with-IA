def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using genetic algorithm."""

    # Create a genetic algorithm object
    ga = funsearch.GA(population_size=100, num_generations=100)

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return 1 / calculate_route_distance(route, _distances)

    # Create the initial population of routes
    population = ga.population_initializer(len(_distances))

    # Run the genetic algorithm
    best_route = ga.evolve(population, fitness)

    # Return the best route
    return best_route
