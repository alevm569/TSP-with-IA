def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize a starting route with the first city
    start_city = 0
    route = [start_city]

    # Create a set of unvisited cities
    unvisited_cities = set(range(len(_distances))) - {start_city}

    # Iterate until all cities are visited
    while unvisited_cities:
        # Find the nearest unvisited city to the last city in the route
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])

        # Add the nearest city to the route
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Return the route, including the return to the starting city
    return route + [start_city]
