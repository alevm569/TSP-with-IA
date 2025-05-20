def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize the starting point as the first city
    current_city = 0
    route = [current_city]

    # Create a list of unvisited cities
    unvisited_cities = list(range(1, len(_distances)))

    # Iteratively find the closest unvisited city and add it to the route
    while unvisited_cities:
        # Find the closest unvisited city to the current city
        closest_city = min(unvisited_cities, key=lambda c: _distances[current_city][c])

        # Add the closest city to the route
        route.append(closest_city)

        # Remove the closest city from the list of unvisited cities
        unvisited_cities.remove(closest_city)

        # Update the current city
        current_city = closest_city

    # Return the route with the starting point appended to the end
    return tuple(route + [route[0]])
