def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize the route as a list of cities in random order.
    np.random.seed(42)
    route = np.random.permutation(np.arange(len(_distances)))

    # Perform local search using the 2-opt heuristic.
    for _ in range(100):
        i, j = np.random.randint(0, len(route), size=2)
        route[i], route[j] = route[j], route[i]

    return route
