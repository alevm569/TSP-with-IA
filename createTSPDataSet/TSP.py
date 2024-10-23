import concurrent
import os
from concurrent.futures import ThreadPoolExecutor
from typing import List, Dict

from createTSPDataSet import current_dir
from createTSPDataSet.TSPSolution import TSPSolution
from createTSPDataSet.TSP_LP.TSP_LP import TSP
from createTSPDataSet.utils.constants import Heuristics, Edges
from createTSPDataSet.utils.generateUtil import generate_cities_with_distances
from createTSPDataSet.utils.nUtil import find_nearest_neighbor_path_solution, find_best_route_2opt
from createTSPDataSet.utils.plotUtil import plot_route

data_path = os.path.join(current_dir, "data")
def get_best_path_nearest_neighbor_and_2opt(cities, distances, seed=123):
    ruta = find_nearest_neighbor_path_solution(cities, distances, seed)
    ruta, distance = find_best_route_2opt(distances, ruta, seed)
    return ruta, distance


def generate_sample(n_cities: int, seed=123, show_name=False):
    cities, distances = generate_cities_with_distances(n_cities, seed)
    solutions = generate_solution_with_heuristics(cities, distances, seed=seed, n_solutions=17)
    min_solution, max_solution, best_edges = get_edges_from_solution(solutions)
    plot_route(cities, distances, max_solution.route, title="Nearest Neighbor + 2-opt Max", show_name=show_name, marked_edges=best_edges)
    plot_route(cities, distances, min_solution.route, title="Nearest Neighbor + 2-opt Min", show_name=show_name, marked_edges=best_edges)

    heuristics = [Heuristics.BestEdges, Heuristics.NearestNeighbour]
    lp_solution = generate_solution_with_lp(cities, distances, heuristics, min_solution, max_solution, best_edges, show_name)
    lp_solution.save_as_pickle(data_path)


def generate_solution_with_heuristics(cities, distances, seed: int = 123, n_solutions: int = 5):
    solutions = []

    def solve(seed_offset):
        new_seed = seed + seed_offset
        route, distance = get_best_path_nearest_neighbor_and_2opt(cities, distances, new_seed)
        return TSPSolution(cities, distances, route, distance)

    with ThreadPoolExecutor() as executor:
        # Execute the solve function with different seeds in parallel
        futures = [executor.submit(solve, offset * 7) for offset in range(n_solutions)]

        # Take the results as they come in
        for future in concurrent.futures.as_completed(futures):
            solutions.append(future.result())
            print(f"Solution {len(solutions)}: {solutions[-1].distance}")

    return solutions


def count_edges_in_solutions(solutions: List[TSPSolution]) -> (TSPSolution, TSPSolution, Dict[str, str]):
    min_solution = solutions[0]
    max_solution = solutions[0]
    edges = dict()
    best_edges = dict()
    for tsp_solution in solutions:
        if tsp_solution.distance < min_solution.distance:
            min_solution = tsp_solution
        if tsp_solution.distance > max_solution.distance:
            max_solution = tsp_solution
        for edge in tsp_solution.edges:
            if edges.get(edge, None) is None:
                edges[edge] = 1
            else:
                edges[edge] += 1
            if edges[edge] == len(solutions):
                (i, j) = edge
                best_edges[i] = j
                best_edges[j] = i
    return min_solution, max_solution, best_edges


# This is quite similar to ACO, edges that appear in all solutions are the best edges
def get_edges_from_solution(solutions: List[TSPSolution]) -> (TSPSolution, TSPSolution, Edges):
    min_solution, max_solution, best_edges = count_edges_in_solutions(solutions)
    edge_result = dict()
    for (i, j) in min_solution.directed_edges:
        if best_edges.get(i, None) == j:
            edge_result[i] = j
    return min_solution, max_solution, edge_result


def generate_solution_with_lp(cities, distances, heuristics: List[Heuristics],
                              min_solution: TSPSolution, max_solution: TSPSolution, best_edges: Edges,
                              show_name: bool = False):
    tsp = TSP(cities, distances, heuristics, best_edges)
    tsp.min_possible_distance = min_solution.distance
    tsp.max_possible_distance = max_solution.distance
    tsp.create_model()
    route = tsp.solve_model(mip_gap=0.01, time_limit_seconds=60, tee=True)
    tsp.plot_results(route, show_name, "TSP with LP")
    return TSPSolution(cities, distances, route, tsp.solution_distance)


if __name__ == "__main__":
    print("Se ha colocado un límite de tiempo de 30 segundos para la ejecución del modelo.")
    # as reference, see nearest neighbor heuristic
    generate_sample(100, show_name=True, seed=567)
