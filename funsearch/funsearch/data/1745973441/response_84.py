def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Use nearest neighbor to generate an initial route.
    start_city = 0
    current_city = start_city
    route = [current_city]
    visited = set([current_city])

    while len(visited) < len(_distances):
        nearest_city = np.argmin(_distances[current_city][[c for c in range(len(_distances)) if c not in visited]])
        route.append(nearest_city)
        visited.add(nearest_city)
        current_city = nearest_city

    # Add the starting city to the end of the route.
    route.append(start_city)

    # Perform local search to improve the route.
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Swap two cities in the route.
            route[i], route[j] = route[j], route[i]

            # Calculate the total distance of the new route.
            total_distance = calculate_route_distance(route, _distances)

            # If the new route is better, keep it.
            if total_distance < calculate_route_distance(route, _distances):
                break

    return tuple(route)
