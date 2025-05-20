def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using the nearest neighbor heuristic."""

    # Initialize the route with the first city
    route = [0]

    # Mark the first city as visited
    visited = set([0])

    # Find the nearest unvisited city for each city in the route
    for _ in range(len(_distances) - 1):
        current_city = route[-1]
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

    # Return to the starting city
    route.append(0)

    return tuple(route)
