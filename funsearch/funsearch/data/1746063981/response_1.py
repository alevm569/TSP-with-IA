def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Perform ant colony optimization (ACO) algorithm
    num_cities = len(_distances)
    aco = funsearch.AntColonyOptimization(num_cities, _distances)

    # Run ACO for a specified number of iterations
    num_iterations = 100
    best_route = None
    best_distance = float('inf')

    for _ in range(num_iterations):
        aco.run_iteration()
        current_best_route = aco.best_route
        current_best_distance = calculate_route_distance(current_best_route, _distances)

        if current_best_distance < best_distance:
            best_distance = current_best_distance
            best_route = current_best_route

    return best_route
