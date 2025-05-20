def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a hybrid heuristic that combines different strategies, such as:
    # - nearest neighbor to generate an initial route
    # - 2-opt to optimize the route
    # - local search to further refine the route

    # Example of a hybrid heuristic:
    initial_route = np.random.permutation(len(_distances))
    optimized_route = funsearch.local_search(initial_route, _distances)

    return optimized_route
