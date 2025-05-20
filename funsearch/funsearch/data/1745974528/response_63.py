def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use the nearest neighbor heuristic to generate an initial route
    current_city = 0
    route = [current_city]
    visited = set([current_city])

    while len(visited) < len(_distances):
        nearest_city = None
        min_distance = float('inf')

        for city in range(len(_distances)):
            if city not in visited:
                distance = _distances[current_city][city]
                if distance < min_distance:
                    nearest_city = city
                    min_distance = distance

        route.append(nearest_city)
        visited.add(nearest_city)
        current_city = nearest_city

    # Add the return trip to the starting city
    route.append(route[0])

    return tuple(route)
