def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Perform a hybrid search using multiple heuristics
    best_route = funsearch.hybrid_search(
        initial_solution=funsearch.nearest_neighbor_heuristic(_distances),
        neighbor_function=funsearch.cheapest_insertion_heuristic(_distances),
        objective_function=funsearch.total_distance_objective(_distances),
        iterations=1000,
        population_size=100,
    )

    return best_route
