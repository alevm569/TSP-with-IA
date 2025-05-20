def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a hybrid approach that combines the nearest neighbor and cheapest insertion heuristics.
    # Initialize the route with the nearest neighbor.
    start_city = 0
    route = [start_city]
    visited = set([start_city])

    # Find the cheapest insertion for each city in the route.
    for _ in range(len(_distances) - 1):
        current_city = route[-1]
        next_city = -1
        min_distance = float('inf')

        for i in range(len(_distances)):
            if i not in visited:
                distance = _distances[current_city][i]
                if distance < min_distance:
                    next_city = i
                    min_distance = distance

        route.append(next_city)
        visited.add(next_city)

    # Close the route by returning to the starting city.
    route.append(start_city)

    return tuple(route)
