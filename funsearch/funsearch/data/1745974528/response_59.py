def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using simulated annealing."""
    # Initialize random route
    route = np.random.permutation(np.arange(len(_distances)))

    # Create a simulated annealing object
    sa = funsearch.SimulatedAnnealing(
        initial_state=route,
        cost_function=calculate_route_distance,
        distance_matrix=_distances,
        iterations=10000,
        temperature=1000,
        cooling_rate=0.99,
    )

    # Run the simulated annealing algorithm
    best_route = sa.run()

    return best_route


def calculate_route_distance(route: tuple[int, ...], distance_matrix: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distance_matrix[route[i]][route[(i + 1) % len(route)]]
    return total_distance
