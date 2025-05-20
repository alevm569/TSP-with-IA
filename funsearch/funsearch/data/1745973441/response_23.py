def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithm."""
    # Create a list of cities
    cities = list(range(len(_distances)))

    # Define the fitness function for the genetic algorithm
    def fitness(route):
        total_distance = 0
        for i in range(len(route)):
            total_distance += _distances[route[i]][route[(i + 1) % len(route)]]
        return 1 / total_distance

    # Create the genetic algorithm solver
    solver = funsearch.GA(
        population_size=100,
        fitness_function=fitness,
        tournament_size=3,
        crossover_rate=0.8,
        mutation_rate=0.1,
    )

    # Run the genetic algorithm solver
    best_route = solver.solve(cities)

    # Return the best route as a tuple of integers
    return best_route
