import json
import os
class StatsByIsland:
    best_solution_number: int
    patience: int
    last_best_result: int | float
    iteration_number: int
    elapsed_time_ms: float
    stats_path: str

    def __init__(self):
        self.ini_values()

    def __str__(self):
        return f"Stats({self.best_solution_number}, {self.patience}, {self.last_best_result}, {self.iteration_number}, {self.elapsed_time_ms})"

    def set_stats_path(self, path: str):
        self.stats_path = path

    def ini_values(self):
        self.best_solution_number = 0
        self.patience = 0 
        self.last_best_result = float('inf')
        self.iteration_number = 0
        self.elapsed_time_ms = 0
        self.stats_path = 'stats.json'

    def to_dict(self):
        return {
        'best_solution_number': self.best_solution_number,
        'patience': self.patience,
        'last_best_result': self.last_best_result,
        'iteration_number': self.iteration_number,
        'elapsed_time_ms': self.elapsed_time_ms
        }


    def write_to_file(self):
        if not hasattr(self, "stats_path"):
            raise ValueError("stats_path no ha sido definido.")
        
        os.makedirs(os.path.dirname(self.stats_path), exist_ok=True)
        with open(self.stats_path, 'w') as file:
            json.dump(self.to_dict(), file, indent=4, ensure_ascii=True)
        print("➡️ Guardando stats en:", self.stats_path)


    def register_stats(self, new_best_solution, new_elapsed_time_ms, version_generated):
        self.best_solution_number += 1
        print("best_solution_number", self.best_solution_number)
        print("new_best_solution", new_best_solution)
        print("last_best_result", self.last_best_result)
        print("self.iteration_number", self.iteration_number)
        print("self.patience", self.patience)
        if self.last_best_result == float('inf') or version_generated is None:
            print("Primera vez")
            # Primera vez
            self.last_best_result = new_best_solution
            self.elapsed_time_ms = new_elapsed_time_ms
            self.patience = 0
            self.iteration_number = version_generated
            self.write_to_file()

            return
        self.last_best_result = new_best_solution
        self.elapsed_time_ms = new_elapsed_time_ms

        if new_best_solution < self.last_best_result:
            self.last_best_result = new_best_solution
            self.elapsed_time_ms = new_elapsed_time_ms
            self.patience = 0
            self.iteration_number = version_generated
            self.write_to_file()

            return

        if new_best_solution == self.last_best_result:
            if new_elapsed_time_ms < self.elapsed_time_ms:
                self.elapsed_time_ms = new_elapsed_time_ms
                self.iteration_number = version_generated
                self.patience += 1
                self.write_to_file()

            return  # No hacer nada si el tiempo no mejora

        # new_best_solution > last_best_result
        if new_best_solution > self.last_best_result:
            if new_elapsed_time_ms < self.elapsed_time_ms:
                self.elapsed_time_ms = new_elapsed_time_ms  # Solo actualizar tiempo si es mejor
                self.iteration_number = version_generated
                self.write_to_file()
            else:
                self.patience += 1  # Solo incrementar paciencia si no mejoró nada
        return
