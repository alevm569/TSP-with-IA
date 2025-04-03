import json
import os

import numpy as np


class StatsProblemManager:
    _instance = None
    best_solution_number: int
    patience:int
    last_best_result: int | float
    iteration_number: int
    elapsed_time_ms: float
    stats_path: str

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(StatsProblemManager, cls).__new__(cls)
            cls._instance.stats_path = 'stats.json'
        return cls._instance

    def set_stats_path(self, stats_path: str):
        self.stats_path = stats_path


    def __init__(self):
        self.ini_values()

    def ini_values(self):
        self.best_solution_number = 0
        self.patience = 0  # this number increase
        self.last_best_result = float('inf')
        self.iteration_number = 0
        self.elapsed_time_ms = 0

    def read_from_file(self):
        if not os.path.exists(self.stats_path):
            self.ini_values()
            return
        # read from a JSON file
        with open(self.stats_path, 'r') as file:
            data = json.load(file)
            if data is None:
                self.ini_values()
                return

            self.best_solution_number = data['best_solution_number']
            self.patience = data['patience']
            self.last_best_result = data['last_best_result']
            self.iteration_number = data['iteration_number']
            self.elapsed_time_ms = data['elapsed_time_ms']

    def to_dict(self):
        return {
            'best_solution_number': self.best_solution_number,
            'patience': self.patience,
            'last_best_result': self.last_best_result,
            'iteration_number': self.iteration_number,
            'elapsed_time_ms': self.elapsed_time_ms
        }

    def write_to_file(self):
        with open(self.stats_path, 'w') as file:
            value = self.to_dict()
            file.write(json.dumps(value, indent=4, ensure_ascii=True))

    def register_stats(self, new_best_solution, new_elapsed_time_ms, version_generated):
        self.best_solution_number += 1
        # TODO: check if new_elapsed_time_ms should be evaluated
        print("best_solution_number", self.best_solution_number)
        print("new_best_solution", new_best_solution)
        print("last_best_result", self.last_best_result)
        print("self.iteration_number", self.iteration_number)
        print("self.patience", self.patience)
        if new_best_solution == abs(self.last_best_result):
            # Primera ejecución: actualizar todo sin validaciones
            self.last_best_result = new_best_solution
            self.elapsed_time_ms = new_elapsed_time_ms
            self.patience = 0
            self.iteration_number = version_generated
            self.write_to_file()
            return
        # Caso 1: Si la nueva solución es mejor, actualizar todo
        if new_best_solution < self.last_best_result:
            self.last_best_result = new_best_solution
            self.elapsed_time_ms = new_elapsed_time_ms
            self.patience = 0  # Reiniciar paciencia
            self.iteration_number = version_generated
            self.write_to_file()
            return

        # Caso 2: Si la solución es igual, actualizar solo si el tiempo es mejor
        if new_best_solution == self.last_best_result:
            if new_elapsed_time_ms < self.elapsed_time_ms:
                self.elapsed_time_ms = new_elapsed_time_ms
                self.iteration_number = version_generated
                self.write_to_file()
            return  # No hacer nada si el tiempo es igual o peor

        # Caso 3: Si la nueva solución es peor
        if new_best_solution > self.last_best_result:
            if new_elapsed_time_ms < self.elapsed_time_ms:
                self.elapsed_time_ms = new_elapsed_time_ms  # Solo actualizar tiempo si es mejor
                self.iteration_number = version_generated
                self.write_to_file()
            else:
                self.patience += 1  # Solo incrementar paciencia si no mejoró nada
            return
statsManager = StatsProblemManager()