def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use a metaheuristic algorithm, such as genetic algorithm or ant colony optimization.
    # These algorithms are known to be effective for solving the TSP problem.
    # Here is an example of using the genetic algorithm:
    from pymoo.algorithms.moo.age import AGEMOEA
    from pymoo.optimize import minimize

    # Define the fitness function
    def fitness_function(route):
        return calculate_route_distance(route, _distances)

    # Define the genetic algorithm
    algorithm = AGEMOEA(pop_size=100, tournament_size=3, seed=1)

    # Run the optimization
    res = minimize(fitness_function, algorithm, ('n_gen', 100))

    # Return the best route
    return res.X[0].astype(int)
