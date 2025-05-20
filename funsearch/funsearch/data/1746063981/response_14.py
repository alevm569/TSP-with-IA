def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Create a set to track visited cities.
    visited = set()

    # Start from the first city.
    current_city = 0

    # Create a list to store the route.
    route = []

    # Iterate until all cities have been visited.
    while len(visited) < len(_distances):
        # Add the current city to the route.
        route.append(current_city)

        # Mark the current city as visited.
        visited.add(current_city)

        # Find the next city to visit.
        best_distance = math.inf
        for next_city in range(len(_distances)):
            if next_city not in visited:
                distance = _distances[current_city][next_city]
                if distance < best_distance:
                    best_distance = distance
                    next_city = next_city

        # Set the current city to the next city.
        current_city = next_city

    # Return to the starting city.
    route.append(route[0])

    return tuple(route)
