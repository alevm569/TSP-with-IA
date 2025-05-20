def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Perform local search using the 2-opt heuristic
    best_route = find_best_route_v1(_distances)
    while True:
        improved = False
        for i in range(len(best_route)):
            for j in range(i + 1, len(best_route)):
                new_route = best_route[:i] + (best_route[j],) + best_route[i+1:j] + (best_route[i],) + best_route[j+1:]
                if calculate_route_distance(new_route, _distances) < calculate_route_distance(best_route, _distances):
                    best_route = new_route
                    improved = True
        if not improved:
            break
    return best_route
