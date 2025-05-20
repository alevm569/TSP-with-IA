def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use the ant colony optimization algorithm (ACO)
    num_cities = len(_distances)
    num_ants = 10
    alpha = 1  # Pheromone strength
    beta = 2  # Heuristic strength
    rho = 0.1  # Pheromone evaporation rate

    # Initialize pheromone matrix
    pheromone_matrix = np.ones((num_cities, num_cities))

    # Run ACO algorithm for 100 iterations
    for i in range(100):
        # Send ants to find routes
        routes = []
        for _ in range(num_ants):
            route = np.random.randint(num_cities)
            unvisited = set(range(num_cities))
            unvisited.remove(route)
            while unvisited:
                # Select next city using pheromone and heuristic information
                probabilities = pheromone_matrix[route] ** alpha * (1 / _distances[route, :])[unvisited] ** beta
                next_city = np.random.choice(list(unvisited), p=probabilities / np.sum(probabilities))
                route.append(next_city)
                unvisited.remove(next_city)
            routes.append(route)

        # Update pheromone matrix
        for route in routes:
            for i in range(num_cities):
                pheromone_matrix[route[i], route[(i + 1) % num_cities]] += 1

    # Return the best route found by ACO
    best_route = routes[np.argmin([calculate_route_distance(r, _distances) for r in routes])]
    return best_route
