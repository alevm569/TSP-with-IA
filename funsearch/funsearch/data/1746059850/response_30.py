def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Perform local search using the 2-opt heuristic
    current_route = tuple(range(len(_distances)))
    best_distance = calculate_route_distance(current_route, _distances)

    while True:
        improved = False

        for i in range(len(_distances)):
            for j in range(i + 1, len(_distances)):
                new_route = reverse_segment(current_route, i, j)
                new_distance = calculate_route_distance(new_route, _distances)

                if new_distance < best_distance:
                    best_distance = new_distance
                    current_route = new_route
                    improved = True

        if not improved:
            break

    return current_route

def reverse_segment(route: tuple[int, ...], start: int, end: int) -> tuple[int, ...]:
    """Reverses the segment of the route between two cities."""
    return route[:start] + route[start:end+1][::-1] + route[end+1:]
