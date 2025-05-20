def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Apply the nearest neighbor heuristic to find an initial route.
    start_city = 0
    current_city = start_city
    route = [current_city]
    visited = set([current_city])

    while len(visited) < len(_distances):
        closest_city = None
        min_distance = float('inf')

        for city in range(len(_distances)):
            if city not in visited:
                distance = _distances[current_city][city]
                if distance < min_distance:
                    min_distance = distance
                    closest_city = city

        route.append(closest_city)
        visited.add(closest_city)
        current_city = closest_city

    # Close the route by returning to the starting city.
    route.append(start_city)

    return tuple(route)
