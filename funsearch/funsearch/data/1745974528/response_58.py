def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using the nearest neighbor heuristic."""

    num_cities = len(_distances)
    visited = set()
    current_city = 0
    route = []

    while len(visited) < num_cities:
        visited.add(current_city)
        next_city = np.argmin(_distances[current_city][:])
        route.append(next_city)
        current_city = next_city

    route.append(route[0])  # Return to the starting city
    return tuple(route)
