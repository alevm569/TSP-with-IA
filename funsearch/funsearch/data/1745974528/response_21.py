def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    This heuristic combines the nearest neighbor strategy for initialization and a 2-opt move operator for local search.
    """

    # Initialize a route using the nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    unvisited_cities = set(range(len(_distances)))
    unvisited_cities.remove(start_city)

    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Perform local search using the 2-opt move operator
    best_route = route
    best_distance = calculate_route_distance(best_route, _distances)

    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:]
            new_route[i:j+1] = new_route[j:i:-1]
            new_distance = calculate_route_distance(new_route, _distances)

            if new_distance < best_distance:
                best_distance = new_distance
                best_route = new_route

    return best_route
