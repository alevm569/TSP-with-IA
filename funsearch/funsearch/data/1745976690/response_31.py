def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or combine multiple heuristics to improve performance.
    # Consider using a hybrid approach that combines different strategies.

    # Example heuristic:
    # Start from an initial city and iteratively add the city that minimizes the distance to the last visited city.

    # Initialize the route with the first city.
    route = [0]
    visited = set([0])

    # Iterate until all cities have been visited.
    while len(visited) < len(_distances):
        # Find the city that minimizes the distance to the last visited city.
        best_city = None
        best_distance = float('inf')

        for i in range(len(_distances)):
            if i not in visited:
                distance = _distances[route[-1]][i]
                if distance < best_distance:
                    best_city = i
                    best_distance = distance

        # Add the best city to the route.
        route.append(best_city)
        visited.add(best_city)

    # Return the best route as a tuple of city indices.
    return tuple(route)
