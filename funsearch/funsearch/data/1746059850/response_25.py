def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Create a set of unvisited cities
    unvisited = set(range(len(_distances)))

    # Start from the first city
    current_city = 0
    route = [current_city]

    # Continue until all cities are visited
    while len(unvisited) > 0:
        # Find the closest unvisited city
        min_distance = float('inf')
        next_city = None
        for city in unvisited:
            distance = _distances[current_city][city]
            if distance < min_distance:
                min_distance = distance
                next_city = city

        # Add the next city to the route and mark it as visited
        route.append(next_city)
        unvisited.remove(next_city)

        # Update the current city
        current_city = next_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
