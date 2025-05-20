def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using 2-opt heuristic."""

    # Generate an initial route using the nearest neighbor heuristic
    route = nearest_neighbor(_distances)

    # Apply 2-opt heuristic iteratively to improve the route
    for _ in range(100):  # Number of iterations for 2-opt heuristic
        route = two_opt(route, _distances)

    return route


def two_opt(route: tuple[int, ...], _distances: np.ndarray) -> tuple[int, ...]:
    """2-opt heuristic to improve a TSP route."""
    best_distance = calculate_route_distance(route, _distances)
    best_route = route.copy()

    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Create a new route by reversing the subroute between i and j
            new_route = route[:i] + route[j:i:-1] + route[j+1:]

            # Calculate the distance of the new route
            new_distance = calculate_route_distance(new_route, _distances)

            # If the new route is shorter, update the best route
            if new_distance < best_distance:
                best_distance = new_distance
                best_route = new_route.copy()

    return best_route


def nearest_neighbor(_distances: np.ndarray) -> tuple[int, ...]:
    """Nearest neighbor heuristic to generate an initial TSP route."""
    num_cities = len(_distances)
    route = [0]  # Start at city 0
    visited = [False] * num_cities

    for _ in range(num_cities - 1):
        current_city = route[-1]
        min_distance = float('inf')
        next_city = None

        for i in range(num_cities):
            if not visited[i] and _distances[current_city][i] < min_distance:
                min_distance = _distances[current_city][i]
                next_city = i

        route.append(next_city)
        visited[next_city] = True

    route.append(0)  # Return to the starting city
    return tuple(route)
