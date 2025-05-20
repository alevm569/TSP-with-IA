def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Apply the ACO algorithm
    num_cities = len(_distances)
    aco = funsearch.AntColonyOptimization(num_cities, _distances)

    # Run the ACO algorithm
    best_route, best_distance = aco.run()

    return best_route
