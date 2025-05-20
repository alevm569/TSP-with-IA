def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""
    # Initialize a random route
    route = np.random.permutation(len(_distances))

    # Perform local search using a 2-opt heuristic
    for _ in range(100):
        i, j = np.random.randint(0, len(_distances), size=2)
        route = swap_edges(route, i, j)

    return route

def swap_edges(route: np.ndarray, i: int, j: int) -> np.ndarray:
    """Swaps two edges in the route."""
    return np.concatenate((route[:i], route[j:i:-1], route[j+1:]))
