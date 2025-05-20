def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2.

    Uses a hybrid heuristic combining nearest neighbor and 2-opt local search.
    """

    # Initialization
    num_cities = len(_distances)
    current_city = 0
    route = [current_city]

    # Nearest neighbor heuristic
    while len(route) < num_cities:
        nearest_city = -1
        min_distance = math.inf
        for i in range(num_cities):
            if i not in route:
                distance = _distances[current_city][i]
                if distance < min_distance:
                    nearest_city = i
                    min_distance = distance
        route.append(nearest_city)
        current_city = nearest_city

    # Local search using 2-opt
    for _ in range(num_cities):
        for i in range(num_cities):
            for j in range(i + 2, num_cities):
                distance_before = _distances[route[i]][route[i+1]] + _distances[route[j]][route[j-1]]
                distance_after = _distances[route[i]][route[j]] + _distances[route[i+1]][route[j-1]]
                if distance_after < distance_before:
                    route[i+1:j] = route[j-1:i:-1]

    return tuple(route)
