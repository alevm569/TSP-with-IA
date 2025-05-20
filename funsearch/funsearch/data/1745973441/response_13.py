def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with hybrid heuristic."""

    # Use a combination of nearest neighbor and cheapest insertion heuristics
    route = nearest_neighbor(_distances)
    cheapest_insertion(route, _distances)

    # Perform local search
    for _ in range(100):
        # Randomly swap two cities in the route
        i, j = np.random.randint(0, len(_distances), 2)
        route[i], route[j] = route[j], route[i]

        # Check if the new route is better
        if calculate_route_distance(route, _distances) < calculate_route_distance(route, _distances):
            pass
        else:
            # If not, reverse the swap
            route[i], route[j] = route[j], route[i]

    return route


def nearest_neighbor(_distances: np.ndarray) -> tuple[int, ...]:
    """Nearest neighbor heuristic."""
    route = [0]
    unvisited = set(range(1, len(_distances)))

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    return route


def cheapest_insertion(route: list[int], _distances: np.ndarray):
    """Cheapest insertion heuristic."""
    for i in range(1, len(_distances)):
        min_distance = float('inf')
        min_city = None

        for city in range(len(_distances)):
            if city not in route:
                distance = _distances[route[-1]][city] + _distances[city][route[0]] - _distances[route[-1]][route[0]]
                if distance < min_distance:
                    min_distance = distance
                    min_city = city

        route.insert(i, min_city)
