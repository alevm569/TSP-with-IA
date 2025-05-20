def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Create a list of candidate routes using the nearest neighbor heuristic.
    candidate_routes = [nearest_neighbor(_distances)]

    # Use a local search algorithm to refine the candidate routes.
    best_route = local_search(candidate_routes[0], _distances)

    return best_route


def nearest_neighbor(_distances: np.ndarray) -> tuple[int, ...]:
    """Constructs a route using the nearest neighbor heuristic."""

    # Start at the first city.
    current_city = 0
    route = [current_city]

    # Visit all other cities.
    remaining_cities = list(range(1, len(_distances)))
    while remaining_cities:
        # Find the nearest city to the current city.
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])

        # Add the nearest city to the route.
        route.append(nearest_city)

        # Remove the nearest city from the list of remaining cities.
        remaining_cities.remove(nearest_city)

        # Update the current city.
        current_city = nearest_city

    return tuple(route)


def local_search(route: tuple[int, ...], _distances: np.ndarray) -> tuple[int, ...]:
    """Refines a route using a local search algorithm."""

    # Iterate until no improvements are found.
    while True:
        # Generate a candidate route by swapping two cities.
        i, j = np.random.randint(0, len(route), 2)
        candidate_route = route[:i] + (route[j],) + route[i+1:j] + (route[i],) + route[j+1:]

        # If the candidate route is better, update the best route.
        if calculate_route_distance(candidate_route, _distances) < calculate_route_distance(route, _distances):
            route = candidate_route
        else:
            break

    return route


def calculate_route_distance(route: tuple[int, ...], _distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""

    distance = 0
    for i in range(len(route)):
        distance += _distances[route[i]][route[(i+1) % len(route)]]

    return distance
