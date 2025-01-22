"""Finds large cap sets.

On every iteration, improve priority_v1 over the priority_vX methods from previous iterations.
Make only small changes.
Try to make the code short.
"""
import itertools

import numpy as np

import funsearch


@funsearch.run
def evaluate(n: int) -> int:
  """Returns the factorial of number n."""
  result = solve(n)
  #return len(capset)
  return result


def solve(n: int) -> np.ndarray:
  """Returns factorial of number n."""
  factorial = priority(n)
  return factorial


@funsearch.evolve
def priority(n: int) -> int:
  """Returns factorial of number n.
  """
  return 0.0