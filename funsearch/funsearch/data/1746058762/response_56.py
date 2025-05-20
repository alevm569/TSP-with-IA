def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Perform local search using the 2-opt heuristic.
    best_route = find_initial_route(_distances)
    while True:
        improved = False
        for i in range(len(best_route)):
            for j in range(i + 1, len(best_route)):
                new_route = swap(best_route, i, j)
                if calculate_route_distance(new_route, _distances) < calculate_route_distance(best_route, _distances):
                    best_route = new_route
                    improved = True
        if not improved:
            break

    # Ensure that the route includes all cities exactly once and returns to the starting point.
    return best_route

def find_initial_route(_distances: np.ndarray) -> tuple[int, ...]:
    """Initializes a route using the nearest neighbor heuristic."""
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    return tuple(route)

def swap(route: tuple[int, ...], i: int, j: int) -> tuple[int, ...]:
    """Swaps two cities in a route."""
    return route[:i] + (route[j],) + route[i + 1:j] + (route[i],) + route[j + 1:]

def calculate_route_distance(route: tuple[int, ...], _distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += _distances[route[i]][route[(i + 1) % len(route)]]
    return distance
