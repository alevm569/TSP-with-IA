def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement your new heuristic here, for example:
    # - Use a genetic algorithm
    # - Apply a local search algorithm with a heuristic
    # - Use a combination of different heuristics

    # Example using a local search algorithm with the nearest neighbor heuristic:
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    route.append(route[0])  # Return to starting city
    return tuple(route)
