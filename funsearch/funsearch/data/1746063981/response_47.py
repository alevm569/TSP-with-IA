def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    This version combines the nearest neighbor heuristic for initialization and the 2-opt local search heuristic for optimization.
    """

    # Initialize route using nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    unvisited = set(range(len(_distances))) - {start_city}

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Optimize route using 2-opt local search
    best_route = route[:]

    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            new_route = route[:i] + route[i+j:] + route[i+j:i+j+2] + route[i:]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(best_route, _distances):
                best_route = new_route

    return best_route
