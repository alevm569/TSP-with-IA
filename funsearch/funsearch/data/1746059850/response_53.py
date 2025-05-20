def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using local search."""

    # Generate an initial random route
    num_cities = len(_distances)
    initial_route = np.random.permutation(num_cities)

    # Create a local search optimizer
    optimizer = funsearch.LocalSearchOptimizer(initial_route)

    # Define the distance function for the TSP problem
    def distance_function(route):
        total_distance = 0
        for i in range(num_cities):
            total_distance += _distances[route[i]][route[(i + 1) % num_cities]]
        return total_distance

    # Run the local search optimizer
    optimal_route = optimizer.optimize(distance_function)

    return optimal_route
