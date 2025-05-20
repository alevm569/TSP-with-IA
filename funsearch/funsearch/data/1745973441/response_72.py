def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Implement a hybrid heuristic that combines different strategies, such as:
    # - Using a nearest neighbor approach to find an initial tour.
    # - Applying a 2-opt neighborhood search to improve the tour.
    # - Incorporating a genetic algorithm to explore diverse solutions.

    # Example hybrid heuristic using nearest neighbor and 2-opt:
    # 1. Find an initial tour using the nearest neighbor heuristic.
    # 2. Apply the 2-opt neighborhood search to improve the tour.
    # 3. Repeat step 2 until no further improvements are found.

    # Return the best route found.
    return best_route
