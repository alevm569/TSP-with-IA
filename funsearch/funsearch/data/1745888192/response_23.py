def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a hybrid approach that combines the nearest neighbor and cheapest insertion heuristics.

    # Use the funsearch library to perform a local search optimization.
    def objective_function(route):
        return calculate_route_distance(route, _distances)

    best_route = funsearch.localsearch(objective_function, tuple(range(len(_distances))))

    # Return the best route as a tuple of city indices.
    return best_route
