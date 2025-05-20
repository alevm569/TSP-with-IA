def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with a new heuristic."""

    # Use a combination of nearest neighbor and cheapest insertion heuristics
    route = np.random.permutation(len(_distances))
    while len(route) < len(_distances):
        # Find the nearest unvisited city
        nearest_city = np.argmin(_distances[route[-1]])
        if nearest_city not in route:
            route = np.append(route, nearest_city)

        # Find the cheapest city to insert
        cheapest_city = np.argmin(_distances[route[-2]][route[-1]])
        route = np.insert(route, -1, cheapest_city)

    return route
