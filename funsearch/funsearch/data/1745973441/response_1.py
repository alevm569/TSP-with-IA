def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a combination of heuristics."""

    # Initialize the route with the first city
    route = [0]

    # Use the nearest neighbor heuristic to find the next city
    current_city = 0
    while len(route) < len(_distances):
        # Find the city that is not in the route and has the shortest distance to the current city
        next_city = -1
        min_distance = float('inf')
        for i in range(len(_distances)):
            if i not in route and _distances[current_city][i] < min_distance:
                next_city = i
                min_distance = _distances[current_city][i]

        # Add the next city to the route
        route.append(next_city)
        current_city = next_city

    # Add the starting city back to the route
    route.append(0)

    return tuple(route)
