def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using 2-opt heuristic."""

    # Initialize a random route
    route = np.random.permutation(len(_distances))

    # Iterate until convergence
    while True:
        # Apply 2-opt heuristic to find two edges to swap
        best_delta = float('inf')
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                delta = _distances[route[i]][route[(j-1)%len(route)]] + _distances[route[(j+1)%len(route)]][route[i]] - _distances[route[i]][route[j]] - _distances[route[(j-1)%len(route)]][route[(j+1)%len(route)]]
                if delta < best_delta:
                    best_delta = delta
                    best_i, best_j = i, j

        # If no improvement is found, return the current route
        if best_delta == float('inf'):
            return route

        # Swap the two edges and update the route
        route[best_i], route[best_j] = route[best_j], route[best_i]
