def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Perform local search optimization using the 2-opt heuristic
    best_route = find_best_route_v1(_distances)
    best_distance = calculate_route_distance(best_route, _distances)

    while True:
        improved = False
        for i in range(len(best_route)):
            for j in range(i + 1, len(best_route)):
                new_route = swap_edges(best_route, i, j)
                new_distance = calculate_route_distance(new_route, _distances)

                if new_distance < best_distance:
                    best_route = new_route
                    best_distance = new_distance
                    improved = True

        if not improved:
            break

    return best_route


def swap_edges(route: tuple[int, ...], i: int, j: int) -> tuple[int, ...]:
    """Swaps two edges in a route."""
    return route[:i] + route[i:j+1][::-1] + route[j+1:]
