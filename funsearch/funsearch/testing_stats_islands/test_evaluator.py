from funsearch.evaluator import Evaluator
from funsearch.testing_stats_islands.FakeProgramDatabase import FakeProgramsDatabase
from funsearch.testing_stats_islands.FakeSandbox import FakeSandbox
from funsearch.testing_stats_islands.FakeTemplate import FakeProgram
from funsearch.StatsProblemManager2 import StatsProblemManager2
import numpy as np


def run_test():
    # Mapas de resultados por input simulados
    output_map = {
        "input1": 120,
        "input2": 80,
        "input3": 200
    }

    sandbox = FakeSandbox(output_map)
    database = FakeProgramsDatabase()
    template = FakeProgram()

    evaluator = Evaluator(
        database=database,
        sbox=sandbox,
        template=template,
        function_to_evolve="find_best_route",
        function_to_run="evaluate",
        inputs=[np.zeros((4, 4))],
        timeout_seconds=10
    )

    sample1 = """```python
    def find_best_route(_distances):
        return tuple(range(len(_distances)))
    ```"""

    sample2 = """```python
    def find_best_route(_distances):
        return (0, 2, 1, 3)
    ```"""

    sample3 = """```python
    def find_best_route(_distances):
        return (0, 3, 2, 1)
    ```"""

    print("🔹 Evaluando isla 1")
    evaluator.analyse(sample1, island_id=1, version_generated=1)

    print("🔹 Evaluando isla 2")
    evaluator.analyse(sample2, island_id=2, version_generated=2)
    print("🔹 Evaluando isla 3")
    evaluator.analyse(sample3, island_id=3, version_generated=3)

    print("\n📊 Estado final:")
    stats_manager = StatsProblemManager2()
    for island_id, stats in stats_manager.solution_by_island.items():
        print(f"Isla {island_id} → Mejor score: {stats.last_best_result}, Tiempo: {stats.elapsed_time_ms}")


if __name__ == "__main__":
    run_test()
