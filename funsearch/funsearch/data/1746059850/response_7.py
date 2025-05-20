def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Apply nearest neighbor heuristic to find an initial solution
    current_city = 0
    route = [current_city]

    while len(route) < len(_distances):
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Apply 2-opt heuristic to improve the solution
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_original = _distances[route[i]][route[j]]

            # Swap the two cities in the route
            route[i:j] = route[j:i:-1]

            # Calculate the new distance
            distance_new = _distances[route[i]][route[j]]

            # If the new distance is shorter, keep the swap
            if distance_new < distance_original:
                continue
            else:
                # Otherwise, restore the original route
                route[i:j] = route[j:i:-1]

    return tuple(route)
