def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize variables
    num_cities = len(_distances)
    best_route = None
    best_distance = float('inf')

    # Generate a set of candidate routes using a nearest neighbor heuristic
    def generate_candidate_routes():
        routes = []
        for start_city in range(num_cities):
            route = [start_city]
            unvisited_cities = set(range(num_cities)) - {start_city}
            while unvisited_cities:
                current_city = route[-1]
                nearest_city = min(unvisited_cities, key=lambda c: _distances[current_city][c])
                route.append(nearest_city)
                unvisited_cities.remove(nearest_city)
            routes.append(route)
        return routes

    # Evaluate the total distance of a route
    def calculate_route_distance(route):
        distance = 0
        for i in range(num_cities):
            distance += _distances[route[i]][route[(i + 1) % num_cities]]
        return distance

    # Run a local search algorithm to find the best route
    for candidate_route in generate_candidate_routes():
        current_route = candidate_route.copy()
        best_distance_for_route = calculate_route_distance(current_route)

        while True:
            # Find two cities to swap in the route
            i, j = np.random.randint(num_cities, size=2)
            current_route[i], current_route[j] = current_route[j], current_route[i]

            # Calculate the new distance of the route
            new_distance = calculate_route_distance(current_route)

            # If the new distance is better, keep the new route
            if new_distance < best_distance_for_route:
                best_distance_for_route = new_distance
            else:
                # Otherwise, revert the swap
                current_route[i], current_route[j] = current_route[j], current_route[i]

            # If the best distance for the route has not improved in a certain number of iterations, stop the local search
            if best_distance_for_route >= best_distance:
                break

        # If the best distance for the current route is better than the overall best distance, update the best route
        if best_distance_for_route < best_distance:
            best_distance = best_distance_for_route
            best_route = current_route

    # Return the best route
    return best_route
