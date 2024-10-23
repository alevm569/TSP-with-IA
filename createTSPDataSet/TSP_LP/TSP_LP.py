import re
from typing import Dict

import pyomo.environ as pyo

from createTSPDataSet.utils.constants import Heuristics
from createTSPDataSet.utils.distanceUtil import *
from createTSPDataSet.utils.nUtil import find_best_route_2opt
from createTSPDataSet.utils.plotUtil import plot_route


class TSP:
    def __init__(self, cities, distances, heuristics: List[str], best_edges: Dict = None):
        self.average_distance = get_average_distance(distances)
        self.max_possible_distance = None
        self.min_possible_distance = None
        self.cities = cities
        self.distances = distances
        self.heuristics = heuristics
        self.average_distance_for_city = get_average_distance_for_city(distances)
        self.model = None
        self.best_edges = best_edges
        self.solution_distance = 0

    def print_min_max_distances(self):
        print(f"Minimum detected possible total distance: {self.min_possible_distance}")
        print(f"Maximum detected possible total distance: {self.max_possible_distance}")
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
        # self.model.be = pyo.Var(self.model.N, self.model.M, within=pyo.Binary)
        self.model.u = pyo.Var(self.model.N, bounds=(0, n_cities - 1), within=pyo.NonNegativeIntegers)

        # Objetive Function: Minimize the total distance
        def obj_rule(model):
            return sum(self.distances[i, j] * model.x[i, j] for i in model.N for j in model.M if i != j)

        self.model.obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)

        # if max possible distance, then limit obj function
        if self.max_possible_distance is not None:
            self.model.obj_upper_bound = pyo.Constraint(expr=self.model.obj <= self.max_possible_distance)

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
                model.x[i, j].fix(0)
                return model.x[i, j] == 0

        self.model.complete_path = pyo.Constraint(self.model.U, self.model.N, rule=rule_forming_the_path)

        # Heurístics:

        if Heuristics.NearestNeighbour in self.heuristics:
            def rule_nearest_neighbour(model, i, j):
                if i == j:
                    return pyo.Constraint.Skip
                if self.average_distance_for_city[i] > self.average_distance:
                     return pyo.Constraint.Skip
                return model.x[i,j] * self.distances[i,j] <= self.average_distance_for_city[i]
            self.model.nearest_neighbor = pyo.Constraint(self.model.N, self.model.M, rule=rule_nearest_neighbour)


        if Heuristics.BestEdges in self.heuristics and self.best_edges is not None:
            # using the idea behind the ACO (Ant Colony Optimization) algorithm
            def rule_best_edges(model, i, j):
                # apply best edges heuristic that select for best edges reducing the solution space
                if self.best_edges.get(i, None) == j:
                    return model.x[i, j] == 1
                if self.best_edges.get(j, None) == i:
                    return model.x[j, i] == 1
                return pyo.Constraint.Skip
            self.model.best_edges = pyo.Constraint(self.model.N, self.model.M, rule=rule_best_edges)
            #print(self.model.best_edges.pprint())
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
        best_route, distance = find_best_route_2opt(self.distances, path)
        self.solution_distance = distance
        print("Total Distance:", distance)
        return best_route

    def plot_results(self, route: List[str], show_name: bool = True, title=None):
        plot_route(self.cities, self.distances, route, show_name=show_name, title=title, marked_edges=self.best_edges)