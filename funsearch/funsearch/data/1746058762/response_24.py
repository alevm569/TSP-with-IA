def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or strategy here.
    # For example, you could use a hybrid approach that combines different heuristics.

    # Use a genetic algorithm to find a good route.
    from pymoo.algorithms.soo.nonconvex.ga import GA
    from pymoo.optimize import minimize

    # Define the fitness function.
    def fitness_function(route: np.ndarray) -> float:
        return calculate_route_distance(route, _distances)

    # Create the genetic algorithm object.
    algorithm = GA()

    # Run the optimization algorithm.
    result = minimize(fitness_function,
                      np.random.permutation(np.arange(len(_distances))),
                      algorithm,
                      ('n_gen', 100),
                      seed=1)

    # Return the best route found.
    return result.X
