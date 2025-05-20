def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Create an initial route by selecting a random starting city and iteratively adding the nearest unvisited city.
    num_cities = len(_distances)
    current_city = np.random.randint(num_cities)
    route = [current_city]

    # Mark the starting city as visited.
    visited = np.zeros(num_cities, dtype=bool)
    visited[current_city] = True

    # Repeat until all cities have been visited.
    while not np.all(visited):
        # Find the nearest unvisited city.
        nearest_city = np.argmin(_distances[current_city][~visited])

        # Add the nearest city to the route.
        route.append(nearest_city)

        # Mark the nearest city as visited.
        visited[nearest_city] = True

        # Update the current city.
        current_city = nearest_city

    # Return to the starting city.
    route.append(route[0])

    return tuple(route)
