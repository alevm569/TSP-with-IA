def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use the nearest neighbor heuristic to initialize a candidate route.
    start_city = 0
    current_city = start_city
    route = [current_city]

    # Mark the starting city as visited.
    visited = np.zeros(len(_distances), dtype=bool)
    visited[start_city] = True

    # Visit all unvisited cities in order of their distance from the current city.
    while not np.all(visited):
        nearest_city = np.argmin(_distances[current_city][~visited])
        route.append(nearest_city)
        visited[nearest_city] = True
        current_city = nearest_city

    # Return to the starting city.
    route.append(start_city)

    return tuple(route)
