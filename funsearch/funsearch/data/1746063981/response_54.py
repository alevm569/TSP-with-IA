def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Create a set to track visited cities
    visited = set()

    # Start from the first city
    current_city = 0

    # Initialize the route
    route = [current_city]

    # Visit all cities except the starting city
    while len(visited) < len(_distances) - 1:
        # Find the closest unvisited city
        nearest_city = None
        min_distance = float('inf')

        for city in range(len(_distances)):
            if city not in visited:
                distance = _distances[current_city][city]
                if distance < min_distance:
                    nearest_city = city
                    min_distance = distance

        # Add the nearest city to the route
        route.append(nearest_city)

        # Mark the city as visited
        visited.add(nearest_city)

        # Update the current city
        current_city = nearest_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
