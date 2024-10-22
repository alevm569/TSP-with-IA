import pyomo.environ as pyo
import re
import sys, os

current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_dir)

from createTSPDataSet.utils.constants import Heuristics
from createTSPDataSet.utils.distanceUtil import *
from createTSPDataSet.utils.generateUtil import generate_cities_with_distances
from createTSPDataSet.utils.nUtil import find_nearest_neighbor_path_solution, find_best_route_2opt
from createTSPDataSet.utils.plotUtil import plot_route
import datetime as dt


class TSP:
    def __init__(self, cities, distances, heuristics: List[str]):
        self.max_possible_distance = None
        self.min_possible_distance = None
        self.cities = cities
        self.distances = distances
        self.heuristics = heuristics
        self.min_distance = get_min_distance(distances)
        self.max_distance = get_max_distance(distances)
        self.average_distance = get_average_distance(distances)
        self.average_distance_for_city = get_average_distance_for_city(distances)
        self.min_distance_for_city = get_min_distances(distances)
        self.max_distance_for_city = get_max_distances(distances)
        self.cal_min_max_distances()
        self.model = None

    def cal_min_max_distances(self):
        medium_low_distance = (self.min_distance + self.average_distance) / 2
        self.min_possible_distance = medium_low_distance * len(self.cities) * 0.25
        self.max_possible_distance = medium_low_distance * len(self.cities) * 0.6


    def print_min_max_distances(self):
        print(f"Distance min between nodes: {self.min_distance}")
        print(f"Distance max between nodes: {self.max_distance}")
        print(f"Average distance between nodes: {self.average_distance}")
        print(f"Minimum possible total distance: {self.min_possible_distance}")
        print(f"Maximum possible total distance: {self.max_possible_distance}")
        print(f"Applied heuristics: {self.heuristics}")

    def create_model(self):
        self.model = pyo.ConcreteModel()

        cities = list(self.cities.keys())
        n_cities = len(cities)

        # Sets to work with
        self.model.M = pyo.Set(initialize=self.cities.keys())
        self.model.N = pyo.Set(initialize=self.cities.keys())

        # Index for the dummy variable u
        self.model.U = pyo.Set(initialize=cities[1:])

        # Variables
        self.model.x = pyo.Var(self.model.N, self.model.M, within=pyo.Binary)
        self.model.u = pyo.Var(self.model.N, bounds=(0, n_cities - 1), within=pyo.NonNegativeIntegers)

        # Objetive Function: (función objetivo a minimizar)
        def obj_rule(model):
            return sum(self.distances[i, j] * model.x[i, j] for i in model.N for j in model.M if i != j)

        self.model.obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)

        # Restrictions
        # One way from each city
        def rule_one_way_from_each_city(model, city_j):
            return sum(model.x[i, city_j] for i in model.N if city_j != i) == 1
        self.model.one_way_i_j = pyo.Constraint(self.model.M, rule=rule_one_way_from_each_city)

        def rule_one_way_to_each_city(model, city_i):
            return sum(model.x[city_i, j] for j in model.M if city_i != j) == 1
        self.model.one_way_j_i = pyo.Constraint(self.model.N, rule=rule_one_way_to_each_city)

        def rule_forming_the_path(model, i, j):
            if i != j:
                return model.u[i] - model.u[j] + model.x[i, j] * n_cities <= n_cities - 1
            else:
                # No same city (self travel)
                return model.x[i, j] == 0

        self.model.complete_path = pyo.Constraint(self.model.U, self.model.N, rule=rule_forming_the_path)

        # Heurístics:

        if Heuristics.NearestNeighbour in self.heuristics:
            def rule_nearest_neighbour(model, i, j):
                if i == j:
                    return pyo.Constraint.Skip
                if self.average_distance_for_city[i] > self.average_distance:
                     return pyo.Constraint.Skip
                return model.x[i,j] * self.distances[i,j] <= (self.average_distance_for_city[i] + self.min_distance_for_city[i]) / 2
            self.model.nearest_neighbor = pyo.Constraint(self.model.N, self.model.M, rule=rule_nearest_neighbour)
        return self

    def solve_model(self, mip_gap, time_limit_seconds, tee):
        if self.model is None:
            return print("Model not created yet")

        # Solving the model
        start_time = dt.datetime.now()
        solver = pyo.SolverFactory('glpk')
        solver.options['mipgap'] = mip_gap
        solver.options['tmlim'] = time_limit_seconds
        results = solver.solve(self.model, tee=tee)

        execution_time = dt.datetime.now() - start_time
        print(f"Execution time: {delta_time_mm_ss(execution_time)}")
        self.print_min_max_distances()

        # Showing the results
        if results.solver.termination_condition == pyo.TerminationCondition.optimal:
            print("Optimal solution found")
        else:
            print("No optimal solution found, but the solver has terminated")

        return self.get_results()

    def get_results(self):
        edges = dict()
        valid_paths = []
        for v in self.model.component_data_objects(pyo.Var):
            if v.domain == pyo.Boolean and v.value is not None and v.value > 0:
                edge = re.search(r'\[(\w\d)*,(\w\d)*]', v.name)
                city1, city2 = edge.group(1), edge.group(2)
                key = f"{city1}_{city2}"
                # Esto evita caer en ciclos cerrados
                if key not in valid_paths:
                    valid_paths += [f"{city1}_{city2}", f"{city2}_{city1}"]
                    edges[city1] = city2

        initial_city = list(self.cities.keys())[0]
        path = get_path(edges, initial_city, [])
        path.append(path[0])
        distance = calculate_path_distance(self.distances, path)
        print("Total Distance:", distance)
        return path

    def plot_results(self, ruta: List[str], show_label: bool = True, title=None):
        plot_route(self.cities, self.distances, ruta, show_label, title)



def get_best_path_nearest_neighbor_and_2opt(cities, distances):
    ruta = find_nearest_neighbor_path_solution(cities, distances)
    ruta = find_best_route_2opt(distances, ruta)
    return ruta



def generate(n_cities:int, seed:int=123):
    show_name = True
    cities, distances = generate_cities_with_distances(n_cities, seed)
    route = get_best_path_nearest_neighbor_and_2opt(cities, distances)
    plot_route(cities, distances, route, show_name, "TSP with NN + 2-opt")
    heuristics = [Heuristics.NearestNeighbour]
    tsp = TSP(cities, distances, heuristics).create_model()
    route = tsp.solve_model(mip_gap=0.05, time_limit_seconds=60, tee=True)
    best_route = find_best_route_2opt(distances, route)
    tsp.plot_results(best_route, show_name, "TSP with LP")


if __name__ == "__main__":
    print("Se ha colocado un límite de tiempo de 30 segundos para la ejecución del modelo.")
    # as reference, see nearest neighbor heuristic
    generate(50)
