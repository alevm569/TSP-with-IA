# Copyright 2023 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""A single-threaded implementation of the FunSearch pipeline."""
import logging

from funsearch import code_manipulation
from funsearch.StatsProblemManager import statsManager
from funsearch.StatsProblemManager2 import statsManager2


def _extract_function_names(specification: str) -> tuple[str, str]:
    """Returns the name of the function to evolve and of the function to run."""
    run_functions = list(
        code_manipulation.yield_decorated(specification, 'funsearch', 'run'))
    if len(run_functions) != 1:
        raise ValueError('Expected 1 function decorated with `@funsearch.run`.')
    evolve_functions = list(
        code_manipulation.yield_decorated(specification, 'funsearch', 'evolve'))
    if len(evolve_functions) != 1:
        raise ValueError('Expected 1 function decorated with `@funsearch.evolve`.')
    return evolve_functions[0], run_functions[0]


def run(samplers, database, iterations: int = -1,  patience_limit: int = 10):
    """Launches a FunSearch experiment."""
    msg = "without limit" if iterations < 0 else f"for {iterations} iterations"
    logging.info(f"Starting FunSearch {msg}")
    current_iteration = 0  # Track the number of completed iterations
    patience = 0

    try:
        # This loop can be executed in parallel on remote sampler machines. As each
        # sampler enters an infinite loop, without parallelization only the first
        # sampler will do any work.
        while iterations != 0:
            current_iteration += 1
            logging.info(f"Iteration {current_iteration}")
            # TODO: Aqui se puede implementar un criterio de parada
            # print(f"StatsManager: {statsManager2.to_dict()}")  # Debugging stats
            # print("best_solution_number", statsManager2.best_solution_number)
            # print("last_best_result", statsManager2.last_best_result)
            # print("self.patience", patience)
            
            if patience >= patience_limit:
                logging.info(f"Stopping early due to patience limit ({patience_limit}) reached, with solution ({statsManager2.best_solution_number}).")
                break

            # Iterate over samplers
            for s in samplers:
                # statics = file()
                # if statics.patience == 20:
                #     break
                s.sample()

            # Update patience based on statsManager
            if statsManager.last_best_result > statsManager.best_solution_number:
                # Found a better solution, reset patience
                statsManager.last_best_result = statsManager.best_solution_number
                patience = 0
            else:
                # No improvement, increment patience
                patience += 1

            # Decrement iterations if a limit is set   
            if iterations > 0:
                iterations -= 1

    except KeyboardInterrupt:
        logging.info("Keyboard interrupt. Stopping.")
    database.backup()
