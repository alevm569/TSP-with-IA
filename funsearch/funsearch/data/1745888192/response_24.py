def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    This function combines the nearest neighbor heuristic to find an initial
    permutation and then uses the 2-opt heuristic to refine it.
    """

    # Initial permutation using nearest neighbor heuristic
    current_city = 0
    route = [current_city]
    visited = set([current_city])

    while len(visited) < len(_distances):
        next_city = np.argmin([_distances[current_city][j] for j in range(len(_distances)) if j not in visited])
        route.append(next_city)
        visited.add(next_city)
        current_city = next_city

    # Refine the route using 2-opt heuristic
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:]
            new_route[i:j+1] = new_route[j:i:-1]

            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return route
