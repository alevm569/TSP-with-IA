def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or hybrid of heuristics here.
    # Consider using genetic algorithms, metaheuristics, or other advanced techniques.

    # Utilize a genetic algorithm to explore the solution space.
    population_size = 100
    num_generations = 100
    tournament_size = 3

    # Create a genetic algorithm object.
    ga = funsearch.GA(population_size, num_generations, tournament_size)

    # Define the fitness function.
    def fitness_function(route):
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm.
    best_route = ga.evolve(fitness_function)

    # Ensure that the returned route satisfies all TSP constraints.
    # - Includes all cities exactly once.
    # - Returns to the starting city.

    return best_route
