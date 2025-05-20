import os
from funsearch.StatsByIsland import StatsByIsland

class Metric:
    best_result: float
    best_time: float

    def __init__(self, best_result: float, best_time: float):
        self.best_result = best_result
        self.best_time = best_time

    def __str__(self):
        return f"{self.best_result}, {self.best_time}"


class StatsProblemManager2:
    _instance = None
    best_solution: StatsByIsland
    solution_by_island: dict[str, StatsByIsland]
    n_best_solution: int
    last_best_metrics: list[Metric]


    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(StatsProblemManager2, cls).__new__(cls)
            cls._instance.best_solution = StatsByIsland()
            cls._instance.best_solution.set_stats_path("stats.json") 
            cls._instance.solution_by_island = {}
        return cls._instance

    def __init__(self):
        self.best_solution = StatsByIsland()
        self.solution_by_island = {}
        self.n_best_solution = 3
        self.last_best_metrics = []

    def get_stats_by_island(self, island_id: str,program_identifier:int) -> StatsByIsland:
        if island_id not in self.solution_by_island:
            stats = StatsByIsland()
            os.makedirs("stats_per_island", exist_ok=True) 
            path = os.path.join("stats_per_island", f"stats_island_{island_id}_{program_identifier}.json")
            stats.set_stats_path(path)
            self.solution_by_island[island_id] = stats
        return self.solution_by_island[island_id]

    def set_stats_by_island(self, island_id: str, stats: StatsByIsland, program_identifier:int):
        self.solution_by_island[island_id] = stats
        path = os.path.join("stats_per_island", f"stats_island_{island_id}_{program_identifier}.json")
        stats.set_stats_path(path)
        stats.write_to_file()
        self.evaluate_best_solution(stats)

    def evaluate_best_solution(self, stats: StatsByIsland):
        if stats.last_best_result < self.best_solution.last_best_result:
            self.best_solution = stats
            self.best_solution.write_to_file()
            return True
        return False


    def is_best_solution(self, metric: Metric) -> bool:
        if len(self.last_best_metrics) == 0:
            return True
        for best_metric in self.last_best_metrics:
            if metric.best_result < best_metric.best_result:
                return True
            if metric.best_result == best_metric.best_result and metric.best_time < best_metric.best_time:
                return True
        return False
    #
    def keep_best_solution(self, metric: Metric) -> bool:
        if self.is_best_solution(metric):
            # keep the n_best_solution in last_best_metrics
            self.last_best_metrics.append(metric)
            if len(self.last_best_metrics) > self.n_best_solution:
                lowest_metric = min(self.last_best_metrics, key=lambda x: x.best_result)
                self.last_best_metrics.remove(lowest_metric)
            return True
        return False

statsManager2 = StatsProblemManager2()

