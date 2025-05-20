def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use the nearest neighbor heuristic to initialize a partial route
    start_city = 0
    current_city = start_city
    route = [current_city]

    # Iterate until all cities have been visited
    while len(route) < len(_distances):
        # Find the nearest unvisited city
        nearest_city = None
        min_distance = math.inf
        for i in range(len(_distances)):
            if i not in route:
                distance = _distances[current_city][i]
                if distance < min_distance:
                    nearest_city = i
                    min_distance = distance

        # Add the nearest city to the route
        route.append(nearest_city)
        current_city = nearest_city

    # Return to the starting city
    route.append(start_city)

    return tuple(route)
