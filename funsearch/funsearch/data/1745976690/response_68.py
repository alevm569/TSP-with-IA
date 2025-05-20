def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a hybrid approach that combines the nearest neighbor heuristic with local search.
    # Initialize a random starting city.
    current_city = np.random.randint(len(_distances))
    route = [current_city]

    # Visit each city exactly once.
    unvisited_cities = set(range(len(_distances))) - {current_city}
    while unvisited_cities:
        # Find the nearest unvisited city.
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)
        current_city = nearest_city

    # Perform local search to improve the route.
    for _ in range(100):
        # Choose two random cities in the route.
        i, j = np.random.randint(len(route), size=2)

        # Swap the two cities.
        route[i], route[j] = route[j], route[i]

        # Check if the route is better.
        if calculate_route_distance(route, _distances) < calculate_route_distance(route, _distances):
            pass  # Keep the new route.
        else:
            route[i], route[j] = route[j], route[i]  # Restore the old route.

    return tuple(route)
