def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Perform local search using the 2-opt heuristic.
    def two_opt(route: list[int]) -> list[int]:
        best_distance = calculate_route_distance(route, _distances)
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                new_route = route[:]
                new_route[i:j] = new_route[j-1:i-1:-1]
                new_distance = calculate_route_distance(new_route, _distances)
                if new_distance < best_distance:
                    best_distance = new_distance
                    route = new_route
        return route

    # Initialize a random route.
    route = list(range(len(_distances)))
    np.random.shuffle(route)

    # Perform local search until convergence.
    while True:
        new_route = two_opt(route)
        if calculate_route_distance(new_route, _distances) == calculate_route_distance(route, _distances):
            break
        route = new_route

    # Return the best route as a tuple of city indices.
    return tuple(route)
