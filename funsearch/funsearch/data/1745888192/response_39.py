def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` with new heuristics.

    This function uses a hybrid approach, combining local search with the 2-opt heuristic.
    It iteratively improves the route by iteratively applying 2-opt and then performing local search.
    """

    # Generate an initial route using the nearest neighbor heuristic
    current_route = nearest_neighbor(_distances)

    # Perform local search and 2-opt iterations
    for _ in range(100):
        # Apply 2-opt to find the best route variation
        best_route = two_opt(current_route, _distances)

        # Perform local search to refine the route
        current_route = local_search(best_route, _distances)

    return current_route

# Helper functions for the new version of find_best_route:

def nearest_neighbor(_distances: np.ndarray) -> tuple[int, ...]:
    # ... (Implementation of nearest neighbor heuristic)

def two_opt(route: tuple[int, ...], _distances: np.ndarray) -> tuple[int, ...]:
    # ... (Implementation of 2-opt heuristic)

def local_search(route: tuple[int, ...], _distances: np.ndarray) -> tuple[int, ...]:
    # ... (Implementation of local search algorithm)
