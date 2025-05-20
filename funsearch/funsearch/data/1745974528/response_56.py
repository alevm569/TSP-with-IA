def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use the nearest neighbor heuristic to generate an initial solution
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        # Find the closest unvisited city
        min_distance = float('inf')
        next_city = None
        for city in remaining_cities:
            distance = _distances[current_city][city]
            if distance < min_distance:
                min_distance = distance
                next_city = city

        # Add the next city to the route and remove it from the remaining cities
        route.append(next_city)
        remaining_cities.remove(next_city)

        # Update the current city
        current_city = next_city

    # Close the loop by returning to the starting city
    route.append(route[0])

    return tuple(route)
