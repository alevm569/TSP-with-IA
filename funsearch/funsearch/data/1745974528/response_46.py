def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or combination of heuristics here.

    # For example, you could use a hybrid approach that combines the nearest neighbor heuristic with the 2-opt local search.

    # Ensure that the route satisfies all constraints, including:
    # - Visiting all cities exactly once
    # - Returning to the starting city

    return tuple(range(len(_distances)))
