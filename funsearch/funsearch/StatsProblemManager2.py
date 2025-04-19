from funsearch.StatsByIsland import StatsByIsland

idx_best = 'best'

class StatsProblemManager2:
    _instance = None
    best_solution: StatsByIsland
    # str is island_id
    solution_by_island: dict[str, StatsByIsland]


    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(StatsProblemManager2, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        self.best_solution = StatsByIsland()


    def get_stats_by_island(self, island_id: str):
        if island_id not in self.solution_by_island:
            self.solution_by_island[island_id] = StatsByIsland()
        return self.solution_by_island[island_id]