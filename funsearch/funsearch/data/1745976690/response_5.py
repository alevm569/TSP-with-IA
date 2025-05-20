def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v1`.

    Uses a hybrid heuristic that combines nearest neighbor and cheapest insertion.
    """

    # Initialize the route with the first city
    route = [0]

    # Create a list of available cities
    available_cities = list(range(1, len(_distances)))

    # Use nearest neighbor to find the first city to add
    current_city = 0
    for _ in range(len(_distances) - 1):
        # Find the nearest available city
        nearest_city = min(available_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        available_cities.remove(nearest_city)
        current_city = nearest_city

    # Add the starting city to complete the cycle
    route.append(0)

    return tuple(route)
