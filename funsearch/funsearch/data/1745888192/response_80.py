def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a hybrid approach that combines the nearest neighbor and cheapest insertion heuristics.

    # Implement the 2-opt heuristic here.
    # The 2-opt heuristic iterates over all pairs of consecutive cities in the route and swaps them if doing so would reduce the total distance.

    # Return the best route as a tuple of city indices.
    return tuple(range(len(_distances)))
