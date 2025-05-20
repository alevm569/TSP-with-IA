def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a hybrid approach that combines different strategies.

    # Example heuristic: Use a greedy approach to find a good initial route.
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(_distances):
        closest_city = -1
        min_distance = float('inf')

        for i in range(len(_distances)):
            if i not in route:
                distance = _distances[current_city][i]
                if distance < min_distance:
                    closest_city = i
                    min_distance = distance

        route.append(closest_city)
        current_city = closest_city

    # Return the route by converting the list to a tuple.
    return tuple(route)
