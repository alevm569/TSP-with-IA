def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Apply the Ant Colony Optimization (ACO) algorithm to find the best route.
    num_cities = len(_distances)
    aco = funsearch.ACO(num_cities, _distances)

    # Run the ACO algorithm for a specified number of iterations.
    best_route = aco.solve()

    return best_route
