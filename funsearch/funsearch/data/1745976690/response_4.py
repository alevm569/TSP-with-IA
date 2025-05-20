def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a metaheuristic algorithm, such as simulated annealing or genetic algorithms,
    # to explore the solution space.

    # Define the initial state of the algorithm.
    current_route = np.random.permutation(len(_distances))

    # Define the objective function to be minimized.
    def objective_function(route):
        return calculate_route_distance(route, _distances)

    # Run the metaheuristic algorithm until convergence or a maximum number of iterations.
    best_route = current_route
    best_distance = objective_function(current_route)

    # Return the best route found.
    return best_route
