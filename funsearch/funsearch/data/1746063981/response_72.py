def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using genetic algorithm.
    """

    # Define the fitness function
    def fitness_function(route):
        total_distance = calculate_route_distance(route, _distances)
        return 1 / total_distance

    # Create a genetic algorithm solver
    solver = funsearch.GA(population_size=50, tournament_size=3, elitism=True)

    # Define the search space
    search_space = funsearch.PermutationSearchSpace(len(_distances))

    # Run the genetic algorithm
    best_route = solver.solve(fitness_function, search_space)

    # Return the best route
    return best_route
