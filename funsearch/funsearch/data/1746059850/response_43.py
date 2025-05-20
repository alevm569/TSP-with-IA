def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    Uses a combination of nearest neighbor and 2-opt operations to find an initial solution.
    Then, performs a local search using the 2-opt heuristic to refine the route.
    """

    # Initialize a random route
    num_cities = len(_distances)
    route = np.random.permutation(num_cities)

    # Apply nearest neighbor to find an initial solution
    current_city = route[0]
    unvisited_cities = set(range(num_cities))
    unvisited_cities.remove(current_city)

    while unvisited_cities:
        nearest_city = min(unvisited_cities, key=lambda c: _distances[current_city][c])
        route = np.append(route, nearest_city)
        unvisited_cities.remove(nearest_city)
        current_city = nearest_city

    # Perform local search using 2-opt
    for i in range(num_cities):
        for j in range(i + 1, num_cities):
            route = funsearch.two_opt(route, _distances)

    return route
