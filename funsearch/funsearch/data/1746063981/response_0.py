def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic called "greedy-insertion"
    def greedy_insertion(distances: np.ndarray) -> list[int]:
        # Initialize an empty route
        route = []
        # Mark all cities as unvisited
        unvisited = list(range(len(distances)))

        # Start from the first city
        current_city = 0
        unvisited.remove(current_city)

        # Iterate until all cities are visited
        while unvisited:
            # Find the city with the shortest distance from the current city
            min_distance = float('inf')
            next_city = None
            for city in unvisited:
                if distances[current_city][city] < min_distance:
                    min_distance = distances[current_city][city]
                    next_city = city

            # Add the next city to the route
            route.append(next_city)
            # Remove the next city from the list of unvisited cities
            unvisited.remove(next_city)

            # Update the current city
            current_city = next_city

        # Return the route as a tuple
        return tuple(route)

    # Use greedy-insertion as the heuristic
    route = greedy_insertion(_distances)

    # Return the route
    return route
