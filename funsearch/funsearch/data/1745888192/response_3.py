def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a combination of nearest neighbor and cheapest insertion heuristics.
    # Start from an initial city and iteratively add the closest unvisited city.
    # Then, insert the next closest city in the route that minimizes the total distance.
    # Repeat until all cities have been visited.

    # Start from the first city.
    current_city = 0
    route = [current_city]

    # Visit all the cities except the starting city.
    unvisited_cities = set(range(1, len(_distances)))

    while unvisited_cities:
        # Find the closest unvisited city.
        closest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])

        # Add the closest city to the route.
        route.append(closest_city)

        # Remove the visited city from the set of unvisited cities.
        unvisited_cities.remove(closest_city)

        # Update the current city.
        current_city = closest_city

    # Return to the starting city.
    route.append(route[0])

    return tuple(route)
