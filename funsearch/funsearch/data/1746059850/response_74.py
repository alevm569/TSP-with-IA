def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a hybrid approach combining the nearest neighbor and 2-opt heuristics
    def nearest_neighbor(distances):
        current_city = 0
        route = [current_city]
        unvisited_cities = set(range(len(distances)))
        unvisited_cities.remove(current_city)

        while unvisited_cities:
            nearest_city = min(unvisited_cities, key=lambda city: distances[current_city][city])
            route.append(nearest_city)
            unvisited_cities.remove(nearest_city)
            current_city = nearest_city

        return route

    def two_opt(route, distances):
        best_distance = calculate_route_distance(route, distances)

        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                reversed_route = route[:i] + route[i:j+1][::-1] + route[j+1:]
                distance = calculate_route_distance(reversed_route, distances)

                if distance < best_distance:
                    best_distance = distance
                    best_route = reversed_route

        return best_route

    # Generate an initial route using the nearest neighbor heuristic
    initial_route = nearest_neighbor(_distances)

    # Apply the 2-opt heuristic iteratively to improve the route
    best_route = initial_route
    for _ in range(10):  # Adjust the number of iterations as needed
        best_route = two_opt(best_route, _distances)

    return best_route
