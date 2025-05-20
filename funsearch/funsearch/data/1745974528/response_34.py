def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or strategy here, such as:

    # 1. Use a genetic algorithm to search for optimal routes.
    # 2. Employ a hybrid approach combining multiple heuristics.
    # 3. Introduce a local search algorithm to refine the current best route.

    # Example using a genetic algorithm:
    from pymoo.algorithms.soo.nonconvex.ga import GA
    from pymoo.optimize import minimize
    from pymoo.problems.traveling_salesman import TSP

    # Create the TSP problem instance
    problem = TSP(_distances)

    # Initialize the GA algorithm
    algorithm = GA()

    # Run the optimization
    res = minimize(problem, algorithm, seed=1)

    # Return the best route found
    return res.X.astype(int)
