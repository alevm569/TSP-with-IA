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

"""Class for evaluating programs proposed by the Sampler."""
import ast
import re
from collections.abc import Sequence
import copy
from datetime import datetime
from typing import Any, Tuple
import textwrap

from funsearch import code_manipulation
from funsearch import programs_database
from funsearch import sandbox
from funsearch.StatsProblemManager import statsManager
from funsearch.code_extractor import CodeExtractor
from funsearch.code_manipulation import Function


"""
  Regex to find all methods named 'priority_vX'.
  With each match, start from the 'def priority_vX(' and continue until there's a new line with any of
  - a new 'def'
  - ` or ' or # without indentation
""" #TODO: vale and Guille, check if this regex is correct METHOD_MATCHER = re.compile(r"```python(.*?)def priority_v(.*)*?(.*?)```python")
# METHOD_MATCHER = re.compile(r"def priority_v\d\(.*?\) -> float:(?:\s*(?:[ \t]*(?!def|#|`|').*(?:\n|$)))+")
METHOD_MATCHER = re.compile(r'```python\n(.*?def .*?)```', re.DOTALL)
METHOD_NAME_MATCHER = re.compile(r"(priority_v\d+|find_best_route_v\d+|find_best_route)")


class _FunctionLineVisitor(ast.NodeVisitor):
    """Visitor that finds the last line number of a function with a given name."""

    def __init__(self, target_function_name: str) -> None:
        self._target_function_name: str = target_function_name
        self._function_end_line: int | None = None

    def visit_FunctionDef(self, node: Any) -> None:  # pylint: disable=invalid-name
        """Collects the end line number of the target function."""
        if self._target_function_name in node.name:
            self._function_end_line = node.end_lineno
        else:
            self._function_end_line = -1
        self.generic_visit(node)

    @property
    def function_end_line(self) -> int:
        """Line number of the final line of function `target_function_name`."""
        assert self._function_end_line is not None  # Check internal correctness.
        return self._function_end_line


def _find_method_implementation(generated_code: str) -> Tuple[str, str]:
    """Find the last 'def priority_vX()' method from generated code."""

    matches = METHOD_MATCHER.findall(generated_code)
    if not matches:
        print("No matches found by METHOD_MATCHER.")
        return "", ""

    # Extract only the last found feature
    last_match = matches[-1].strip()
    function_name = METHOD_NAME_MATCHER.search(last_match).group()
    filter_function, final_function_name = CodeExtractor(last_match, function_name).extract()
    return filter_function, final_function_name


def _trim_function_body(generated_code: str) -> str:
    """Extracts the body of the generated function, trimming anything after it."""
    if not generated_code:
        return ''
    if not type(generated_code) is str:
        generated_code = str(generated_code)

    method_name = "fake_function_header"
    # Check is the response only a continuation for our prompt or full method implementation with header
    if any(keyword in generated_code for keyword in ["def priority_v", "def find_best_route", "find_best_route_v"]):
        code, method_name = _find_method_implementation(generated_code)
        if code is None:
            print("No valid code found.")
            return ''
    else:
        code = f'def {method_name}():\n{generated_code}'

    if not code.strip().startswith("def"):
        print("No valid function definition found in the code.")
        return ''

    # Finally parse the code to make sure it's valid Python
    tree = None
    # We keep trying and deleting code from the end until the parser succeeds.
    while tree is None:
        try:
            tree = ast.parse(code)
        except SyntaxError as e:
            print(f"SyntaxError at line {e.lineno}: {e.msg}")
            code = '\n'.join(code.splitlines()[:e.lineno - 1])
            if not code:
                # Nothing could be saved from `generated_code`
                print("No valid code could be parsed.")
                return ''

    visitor = _FunctionLineVisitor(method_name)
    visitor.visit(tree)
    body_lines = code.splitlines()[1:visitor.function_end_line]
    if not body_lines:
        print("No valid body lines found in the function.")
        return ''
    trimmed_body = '\n'.join(body_lines) + '\n\n'
    return trimmed_body

def _sample_to_program(
        generated_code: str,
        version_generated: int | None,
        template: code_manipulation.Program,
        function_to_evolve: str,
) -> tuple[None, None] | tuple[Function, str]:
    """Returns the compiled generated function and the full runnable program."""
    generated_code = ''.join(generated_code)
    body = _trim_function_body(generated_code)
    if len(body) == 0:
        print("Should not continue")
        return None, None
    if version_generated is not None:
        body = code_manipulation.rename_function_calls(
            body,
            f'{function_to_evolve}_v{version_generated}',
            function_to_evolve)

    program = copy.deepcopy(template)
    evolved_function = program.get_function(function_to_evolve)
    evolved_function.body = body
    return evolved_function, str(program)


def _calls_ancestor(program: str, function_to_evolve: str) -> bool:
    """Returns whether the generated function is calling an earlier version."""
    for name in code_manipulation.get_functions_called(program):
        # In `program` passed into this function the most recently generated
        # function has already been renamed to `function_to_evolve` (wihout the
        # suffix). Therefore any function call starting with `function_to_evolve_v`
        # is a call to an ancestor function.
        if name.startswith(f'{function_to_evolve}_v'):
            return True
    return False


class Evaluator:
    """Class that analyses functions generated by LLMs."""

    def __init__(
            self,
            database: programs_database.ProgramsDatabase,
            sbox: sandbox.DummySandbox,
            template: code_manipulation.Program,
            function_to_evolve: str,
            function_to_run: str,
            inputs: Sequence[Any],
            timeout_seconds: int = 30,
    ):
        self._database = database
        self._template = template
        self._function_to_evolve = function_to_evolve
        self._function_to_run = function_to_run
        self._inputs = inputs
        self._timeout_seconds = timeout_seconds
        self._sandbox = sbox

    def analyse(
            self,
            sample: str,
            island_id: int | None,
            version_generated: int | None,
    ) -> None:
        """Compiles the sample into a program and executes it on test inputs."""
        new_function, program_str = _sample_to_program(
            sample, version_generated, self._template, self._function_to_evolve)
        if new_function is None or program_str is None:
            print("Nothing to do")
            return

        statsManager.read_from_file()
        scores_per_test = {}
        stats_per_test = {}
        # inputs -> input sequence
        for current_input in self._inputs:
            start_time = datetime.now()
            # test_output -> is the solution of problem (TSP, shortest path)
            test_output, runs_ok = self._sandbox.run(
                program_str, self._function_to_run, current_input, self._timeout_seconds)
            delta_time = (datetime.now() - start_time).microseconds / 1000
            print(f"Sandbox output: {test_output}, Runs OK: {runs_ok}")

            if not runs_ok:
                continue

            if (runs_ok and not _calls_ancestor(program_str, self._function_to_evolve)
                    and test_output is not None):
                if not isinstance(test_output, (int, float)):
                    raise ValueError('@function.run did not return an int/float score.')
                scores_per_test[current_input] = test_output
                stats_per_test[current_input] = {
                    'test_output': test_output,
                    'delta_time': delta_time
                }
        if scores_per_test:
            # TODO: Register stats for the program.
            # get best score, test_output and delta_time (the lowest)
            best_score = min(stats_per_test.values(), key=lambda x: x['test_output'])
            statsManager.register_stats(best_score['test_output'], best_score['delta_time'], version_generated)
            # if best_score['test_output'] < statsManager.last_best_result:
            if island_id != None:
                self._database.save_best_programs(program_str, best_score['test_output'], island_id)
            self._database.register_program(new_function, island_id, scores_per_test)
