def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a hybrid of the nearest neighbor and cheapest insertion heuristics.

    # Example heuristic:
    current_city = 0
    route = [current_city]
    visited = set([current_city])

    while len(visited) < len(_distances):
        # Find the closest unvisited city
        nearest_city = None
        min_distance = float('inf')

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

    # Return the route
    return tuple(route)
