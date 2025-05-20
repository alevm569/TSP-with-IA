def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with new heuristic."""

    # Implement a new heuristic here, for example:
    def new_heuristic(current_city, unvisited_cities):
        # Calculate the minimum distance to an unvisited city from the current city.
        min_distance = float('inf')
        best_city = None
        for city in unvisited_cities:
            distance = _distances[current_city][city]
            if distance < min_distance:
                min_distance = distance
                best_city = city
        return best_city

    # Use the new heuristic in your route-finding algorithm.
    # ...

    # Return the best route found.
    return route
