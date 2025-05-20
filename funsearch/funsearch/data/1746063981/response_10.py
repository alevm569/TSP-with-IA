def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a genetic algorithm."""

    # Define the fitness function to minimize route distance
    def fitness_function(route):
        return calculate_route_distance(route, _distances)

    # Create a genetic algorithm optimizer
    optimizer = funsearch.GAOptimizer(fitness_function)

    # Set the search space to be all permutations of the cities
    search_space = funsearch.PermutationSearchSpace(len(_distances))

    # Run the optimizer to find the best route
    best_route = optimizer.optimize(search_space)

    return best_route
