def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid approach."""

    # Use the nearest neighbor heuristic to initialize a partial route
    current_city = 0
    route = [current_city]
    unvisited_cities = set(range(1, len(_distances)))

    # Perform local search iteratively
    for _ in range(len(_distances)):
        # Find the nearest unvisited city
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])

        # Add the nearest city to the route and mark it as visited
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)
        current_city = nearest_city

    # Close the route by returning to the starting city
    route.append(route[0])

    return tuple(route)
