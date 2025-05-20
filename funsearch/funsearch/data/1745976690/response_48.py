def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid approach."""

    # Create a list of candidate routes using the nearest neighbor heuristic
    routes = [nearest_neighbor(_distances)]

    # Perform local search on each candidate route to find the best permutation
    best_route = local_search(routes[0], _distances)

    return best_route


def nearest_neighbor(_distances: np.ndarray) -> tuple[int, ...]:
    """Finds a route using the nearest neighbor heuristic."""
    num_cities = len(_distances)
    route = []
    unvisited = set(range(num_cities))

    # Start from the first city
    current_city = 0
    unvisited.remove(current_city)

    # Visit each city in order of their distance from the current city
    while unvisited:
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)
        current_city = nearest_city

    # Return to the starting city
    route.append(0)

    return tuple(route)


def local_search(route: tuple[int, ...], _distances: np.ndarray) -> tuple[int, ...]:
    """Performs local search on a candidate route."""

    # Generate all possible 2-opt swaps
    swaps = [(i, j) for i in range(len(route)) for j in range(i + 1, len(route))]

    # Evaluate each swap and select the best one
    best_route = route
    best_distance = calculate_route_distance(route, _distances)

    for i, j in swaps:
        new_route = route[:i] + route[j:i:-1] + route[j + 1:]
        new_distance = calculate_route_distance(new_route, _distances)

        if new_distance < best_distance:
            best_route = new_route
            best_distance = new_distance

    return best_route
