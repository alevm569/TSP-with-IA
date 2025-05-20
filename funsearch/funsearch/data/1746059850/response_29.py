def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use the Ant Colony Optimization (ACO) algorithm to find the best route.
    num_cities = len(_distances)
    aco = funsearch.AntColonyOptimization(num_cities, _distances)

    # Run the ACO algorithm for a specified number of iterations.
    num_iterations = 1000
    best_route = aco.run(num_iterations)

    return best_route
