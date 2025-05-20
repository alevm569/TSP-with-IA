def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use the nearest neighbor heuristic to generate an initial solution
    current_city = 0
    route = [current_city]
    visited = set([current_city])

    while len(visited) < len(_distances):
        nearest_city = None
        min_distance = float('inf')

        # Find the nearest unvisited city
        for i in range(len(_distances)):
            if i not in visited:
                distance = _distances[current_city][i]
                if distance < min_distance:
                    nearest_city = i
                    min_distance = distance

        # Add the nearest city to the route and mark it as visited
        route.append(nearest_city)
        visited.add(nearest_city)

        # Update the current city
        current_city = nearest_city

    # Close the route by adding the starting city
    route.append(route[0])

    return tuple(route)
