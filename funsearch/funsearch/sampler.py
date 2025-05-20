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

"""Class for sampling new programs."""
from collections.abc import Collection, Sequence

import llm
import numpy as np

from funsearch import evaluator
from funsearch import programs_database
from ollama import OLLAMA
import textwrap
import re

class LLM:
  """Language model that predicts continuation of provided source code."""

  #def __init__(self, samples_per_prompt: int, model: llm.Model, log_path=None) -> None:
  def __init__(self, samples_per_prompt: int, model: OLLAMA, log_path=None) -> None:
    self._samples_per_prompt = samples_per_prompt
    self.model = model
    self.prompt_count = 0
    self.log_path = log_path

  def _draw_sample(self, prompt: str) -> str:
    """Returns a predicted continuation of `prompt`."""
    response = self.model.prompt(prompt)
    to_print = response[0] if isinstance(response, list) else response
    self._log(prompt, response, self.prompt_count)
    self.prompt_count += 1
    return response

  def draw_samples(self, prompt: str) -> Collection[str]:
    """Returns multiple predicted continuations of `prompt`."""
    return [self._draw_sample(prompt) for _ in range(self._samples_per_prompt)]

  def _log(self, prompt: str, response: str, index: int):
    if self.log_path is not None:
      with open(self.log_path / f"prompt_{index}.log", "a") as f:
        f.write(prompt)
      # with open(self.log_path / f"response_{index}.log", "a") as f:
      #   f.write(str(response))
      # Concatenar el response si es una lista
      if isinstance(response, list):
          response = "\n".join(response)

      # Extraer el código Python del response
      code_block = ""
      match = re.search(r"```python\n(.*?)```", response, re.DOTALL)
      if match:
          code_block = match.group(1)
          # Normalizar la indentación del código
          code_block = textwrap.dedent(code_block).strip()

      # Si no se encuentra un bloque de código, guardar el response completo
      if not code_block:
          code_block = response

      # Guardar el código o el response formateado
      with open(self.log_path / f"response_{index}.py", "w", encoding="utf-8") as f:
          f.write(code_block)
          f.write("\n")


class Sampler:
  """Node that samples program continuations and sends them for analysis."""

  def __init__(
      self,
      database: programs_database.ProgramsDatabase,
      evaluators: Sequence[evaluator.Evaluator],
      model: LLM,
  ) -> None:
    self._database = database
    self._evaluators = evaluators
    self._llm = model

  def sample(self):
    """Continuously gets prompts, samples programs, sends them for analysis."""
    prompt = self._database.get_prompt()
    samples = self._llm.draw_samples(prompt.code)
    # This loop can be executed in parallel on remote evaluator machines.
    for sample in samples:
      chosen_evaluator = np.random.choice(self._evaluators)
      chosen_evaluator.analyse(
          sample, prompt.island_id, prompt.version_generated)
