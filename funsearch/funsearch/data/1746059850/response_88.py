def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement your new heuristic here.
    # Consider using a combination of different heuristics or implementing a completely new one.

    # Return the best route as a tuple of city indices.
    return tuple(range(len(_distances)))
