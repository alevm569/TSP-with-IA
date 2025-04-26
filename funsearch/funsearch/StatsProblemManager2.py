import os
from funsearch.StatsByIsland import StatsByIsland

class StatsProblemManager2:
    _instance = None
    best_solution: StatsByIsland
    solution_by_island: dict[str, StatsByIsland]

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(StatsProblemManager2, cls).__new__(cls)
            cls._instance.best_solution = StatsByIsland()
            cls._instance.best_solution.set_stats_path("stats_global.json") 
            cls._instance.solution_by_island = {}
        return cls._instance

    def __init__(self):
        self.best_solution = StatsByIsland()
        self.solution_by_island = {}

    def get_stats_by_island(self, island_id: str) -> StatsByIsland:
        if island_id not in self.solution_by_island:
            stats = StatsByIsland()
            os.makedirs("stats_per_island", exist_ok=True) 
            path = os.path.join("stats_per_island", f"stats_island_{island_id}.json")
            stats.set_stats_path(path)
            self.solution_by_island[island_id] = stats
        return self.solution_by_island[island_id]


statsManager2 = StatsProblemManager2()

