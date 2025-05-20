def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Advanced version of `find_best_route_v1` using simulated annealing."""

    # Initialize the simulated annealing algorithm
    sa = funsearch.SimulatedAnnealing(temp=1000, cooling_rate=0.99)

    # Define the objective function to minimize
    def route_distance(route):
        return calculate_route_distance(route, _distances)

    # Generate a random initial solution
    initial_route = np.random.permutation(len(_distances))

    # Run the simulated annealing algorithm
    best_route = sa.optimize(route_distance, initial_route)

    # Return the best route found
    return best_route
