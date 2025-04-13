import concurrent
import traceback
from concurrent.futures import ThreadPoolExecutor
from typing import List, Dict

from TSP import get_best_path_nearest_neighbor_and_2opt
from TSPSolution import TSPSolution
from TSP_LP.TSP_LP import TSP
from create_loggers import tsp_logger, tsp_detail_logger
from utils.constants import Cities, Heuristics, Edges


def generate_solution_with_heuristics(cities, distances, seed: int = 123, n_solutions: int = 5, verbose: bool = False) -> List[TSPSolution]:
    try:
        return _generate_solution_with_heuristics(cities, distances, seed, n_solutions, verbose)
    except Exception as e:
        tb = traceback.format_exc()
        msg = f"There was an error while generating a solution with heuristics: {e}"
        tsp_logger.error(msg)
        tsp_detail_logger.error(f"{msg}\n{tb}")
        route, distance = get_best_path_nearest_neighbor_and_2opt(cities, distances, 123)
        route2, distance2 = get_best_path_nearest_neighbor_and_2opt(cities, distances, 456)
        return [TSPSolution(cities, distances, route, distance), TSPSolution(cities, distances, route2, distance2)]


def _generate_solution_with_heuristics(cities, distances, seed: int = 123, n_solutions: int = 5, verbose: bool = False) -> List[TSPSolution]:
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
            verbose and tsp_logger.debug(f"Solution {len(solutions)}: {solutions[-1].distance}")

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
def get_edges_from_solution(solutions: List[TSPSolution]) -> (TSPSolution, TSPSolution, Dict[str, str]):
    min_solution, max_solution, best_edges = count_edges_in_solutions(solutions)
    edge_result = dict()
    for (i, j) in min_solution.directed_edges:
        if best_edges.get(i, None) == j:
            edge_result[i] = j
    return min_solution, max_solution, edge_result

def get_solution_with_lp(cities: Cities, distances, heuristics: List[Heuristics],
                              min_solution: TSPSolution, max_solution: TSPSolution, best_edges: Edges,
                              show_name: bool = False, show_plot: bool = False, verbose: bool = False) -> TSPSolution | None:
    try:
        resp = generate_solution_with_lp(cities, distances, heuristics, min_solution, max_solution, best_edges, show_name, show_plot, verbose)
        if not isinstance(resp, TSPSolution):
            return None
        return resp
    except Exception as e:
        tb = traceback.format_exc()
        msg = f"There was an error while generating a solution with LP: {e}"
        tsp_logger.error(msg)
        tsp_detail_logger.error(f"{msg}\n{tb}")
        return None

def generate_solution_with_lp(cities: Cities, distances, heuristics: List[Heuristics],
                              min_solution: TSPSolution, max_solution: TSPSolution, best_edges: Edges,
                              show_name: bool = False, show_plot: bool = False, verbose: bool = False) -> TSPSolution:
    tsp = TSP(cities, distances, heuristics)
    tsp.min_possible_distance = min_solution.distance
    tsp.max_possible_distance = max_solution.distance
    tsp.best_edges = best_edges
    tsp.create_model()
    route = tsp.solve_model(mip_gap=0.0001, time_limit_seconds=60, tee=verbose)
    if show_plot:
        tsp.plot_results(route, show_name, "TSP with LP")
    return TSPSolution(cities, distances, route, tsp.solution_distance)