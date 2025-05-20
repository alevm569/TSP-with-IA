def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Implement a hybrid heuristic that combines different approaches.
    # For example, you could use a nearest neighbor heuristic to generate an initial route,
    # then apply a 2-opt heuristic to improve it.

    # Generate an initial route using a nearest neighbor heuristic.
    current_city = 0
    route = [current_city]
    while len(route) < len(_distances):
        nearest_city = find_nearest_city(current_city, _distances, route)
        route.append(nearest_city)
        current_city = nearest_city

    # Apply a 2-opt heuristic to improve the route.
    funsearch.two_opt(route, _distances)

    return route

def find_nearest_city(current_city: int, distances: np.ndarray, route: list[int]) -> int:
    """Finds the nearest city to the current city that is not already in the route."""
    min_distance = math.inf
    nearest_city = None
    for i in range(len(distances)):
        if i != current_city and i not in route:
            distance = distances[current_city][i]
            if distance < min_distance:
                min_distance = distance
                nearest_city = i
    return nearest_city
