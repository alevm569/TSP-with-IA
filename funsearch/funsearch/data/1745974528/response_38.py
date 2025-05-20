def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use genetic algorithm to find the best route
    from pymoo.algorithms.soo.nonconvex.ga import GA
    from pymoo.optimize import minimize
    from pymoo.problems.traveling_salesman import TSP

    # Create a TSP problem instance
    problem = TSP(distance_matrix=_distances)

    # Create a GA algorithm
    algorithm = GA(pop_size=100, n_gen=100)

    # Run the optimization
    res = minimize(problem, algorithm, seed=1)

    # Return the best route
    return res.X.astype(int)
