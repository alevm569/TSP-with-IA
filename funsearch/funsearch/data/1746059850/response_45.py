def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a hybrid approach combining nearest neighbor and 2-opt heuristics
    route = find_nearest_neighbor_route(_distances)
    route = two_opt_optimize(route, _distances)

    return route

def find_nearest_neighbor_route(_distances: np.ndarray) -> tuple[int, ...]:
    # ... implementation of nearest neighbor heuristic ...

def two_opt_optimize(route: tuple[int, ...], _distances: np.ndarray) -> tuple[int, ...]:
    # ... implementation of 2-opt heuristic ...
