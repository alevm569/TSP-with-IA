def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2`.

    Implement a new heuristic or combine multiple heuristics to improve performance.
    Consider using a hybrid approach that combines different strategies.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Example heuristic:
    # Start from an arbitrary city and iteratively find the nearest unvisited city.

    current_city = 0
    route = [current_city]
    unvisited_cities = set(range(len(_distances)))
    unvisited_cities.remove(current_city)

    while unvisited_cities:
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)
        current_city = nearest_city

    route.append(route[0])  # Return to starting city

    return tuple(route)
