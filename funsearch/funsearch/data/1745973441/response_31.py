def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Generate an initial solution using the nearest neighbor heuristic.
    initial_route = nearest_neighbor(_distances)

    # Apply 2-opt local search to refine the solution.
    best_route = local_search(_distances, initial_route)

    return best_route

def nearest_neighbor(_distances: np.ndarray) -> tuple[int, ...]:
    """Generates an initial route using the nearest neighbor heuristic."""
    num_cities = len(_distances)
    unvisited = set(range(num_cities))
    current_city = np.random.choice(num_cities)
    route = [current_city]

    while unvisited:
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        unvisited.remove(nearest_city)
        route.append(nearest_city)
        current_city = nearest_city

    return tuple(route)

def local_search(_distances: np.ndarray, route: tuple[int, ...]) -> tuple[int, ...]:
    """Applies 2-opt local search to refine a route."""
    best_route = route

    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            new_route = route[:i] + route[i:j][::-1] + route[j:]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(best_route, _distances):
                best_route = new_route

    return best_route
