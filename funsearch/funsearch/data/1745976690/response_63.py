def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a metaheuristic approach."""

    # Initialize a metaheuristic optimizer
    optimizer = funsearch.SimulatedAnnealing(
        fun=calculate_route_distance,
        x0=np.random.permutation(len(_distances)),
        T=1000,
        alpha=0.99,
    )

    # Run the metaheuristic optimization
    optimizer.optimize()

    # Return the best route found
    return tuple(optimizer.x)
