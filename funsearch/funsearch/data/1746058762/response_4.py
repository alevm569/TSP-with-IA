def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a combination of nearest neighbor and local search heuristics
    initial_route = nearest_neighbor(_distances)
    best_route = local_search(initial_route, _distances)

    return best_route


def nearest_neighbor(_distances: np.ndarray) -> tuple[int, ...]:
    """Nearest neighbor heuristic to generate an initial route."""
    num_cities = len(_distances)
    unvisited = set(range(num_cities))
    route = [0]  # Start from city 0

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    return tuple(route)


def local_search(route: tuple[int, ...], _distances: np.ndarray) -> tuple[int, ...]:
    """Local search heuristic to improve an initial route."""
    best_route = route

    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:i] + (route[j],) + route[i + 1:j] + (route[i],) + route[j + 1:]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(best_route, _distances):
                best_route = new_route

    return best_route
