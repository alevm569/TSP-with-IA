def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Apply nearest neighbor heuristic to find an initial route.
    current_city = 0
    route = [current_city]
    visited = set([current_city])

    while len(visited) < len(_distances):
        # Find the closest unvisited city.
        closest_city = None
        min_distance = float('inf')

        for i in range(len(_distances)):
            if i not in visited:
                distance = _distances[current_city][i]
                if distance < min_distance:
                    closest_city = i
                    min_distance = distance

        # Add the closest city to the route and mark it as visited.
        route.append(closest_city)
        visited.add(closest_city)
        current_city = closest_city

    # Add the starting city to the end of the route to complete the cycle.
    route.append(route[0])

    return tuple(route)
