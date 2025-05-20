def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with a hybrid approach."""

    # Initialize route using nearest neighbor heuristic
    current_city = 0
    route = [current_city]
    visited = set([current_city])

    # Iterate until all cities are visited
    while len(visited) < len(_distances):
        # Find the city with the shortest distance to the current city
        min_distance = float('inf')
        next_city = None
        for i in range(len(_distances)):
            if i not in visited:
                distance = _distances[current_city][i]
                if distance < min_distance:
                    min_distance = distance
                    next_city = i

        # Add the next city to the route and mark it as visited
        route.append(next_city)
        visited.add(next_city)

        # Update the current city
        current_city = next_city

    # Close the route by returning to the starting city
    route.append(route[0])

    return tuple(route)
