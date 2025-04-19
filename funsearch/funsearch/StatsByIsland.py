
class StatsByIsland:
    best_solution_number: int
    patience: int
    last_best_result: int | float
    iteration_number: int
    elapsed_time_ms: float
    stats_path: str


    def __init__(self):
        self.ini_values()

    def ini_values(self):
        self.best_solution_number = 0
        self.patience = 0  # this number increase
        self.last_best_result = float('inf')
        self.iteration_number = 0
        self.elapsed_time_ms = 0

    # def read_from_file(self, stats_path: str):

    # def save_to_file(self, stats_path: str):