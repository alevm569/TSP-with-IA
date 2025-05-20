def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2`.

    Uses a hybrid heuristic that combines nearest neighbor and 2-opt operations.
    """

    # Initialize the route with the first city
    route = [0]

    # Find the nearest city to the starting city and add it to the route
    for i in range(1, len(_distances)):
        nearest_city = find_nearest_city(route[-1], _distances)
        route.append(nearest_city)

    # Perform 2-opt operations to improve the route
    for _ in range(len(_distances)):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                distance_before = calculate_route_distance(route, _distances)
                route = two_opt(route, i, j)
                distance_after = calculate_route_distance(route, _distances)
                if distance_after < distance_before:
                    break

    return tuple(route)

# Helper functions for find_best_route_vx

def find_nearest_city(current_city: int, distances: np.ndarray) -> int:
    """Finds the nearest city to the given city."""
    distances_from_current_city = distances[current_city]
    return np.argmin(distances_from_current_city)

def two_opt(route: list[int], i: int, j: int) -> list[int]:
    """Performs a 2-opt operation on the given route."""
    route[i:j+1] = route[j:i-1:-1]
    return route

def calculate_route_distance(route: list[int], distances: np.ndarray) -> float:
    """Calculates the total distance of the given route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i+1) % len(route)]]
    return total_distance
