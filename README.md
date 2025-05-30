
# TSP-with-IA
This project explores solving the Traveling Salesman Problem (TSP) using LLMs (Large Language Models) and the FunSearch evolutionary framework. The generated heuristics are compared against classical solvers like Linear Programming (LP) and Ant Colony Optimization (ACO), which serve as ground truth.


### Objective
To automatically evolve Python functions that solve TSP instances based on distance matrices. These functions are evaluated and ranked according to how close their solutions are to reference solvers.

Project Workflow
1. 📦 Dataset Generation
Run TSP.py to generate TSP instances with n = 20, 50, or 100 cities.

These instances are stored as .pkl files.

Each file includes:

cities, distances, route, matrix_distances, etc.

2. 🎯 Ground Truth Solutions
Run checkProblem.py to compute solutions using LP and ACO.

Ensure utils.py imports the correct constant:

from funsearch.funsearch.constants import n_cities_graph

 ⚙️ Program Evolution via FunSearch
Evolve solutions using FunSearch, guided by a prompt defined in tsp_spec.py.

Before running, make sure:

In utils.py, you import:
from funsearch.constants import n_cities_graph

In constants.py, set:

n_cities_graph = 20  # or 50, or 100 depending on the experiment
Each evolution run produces 3 candidate programs (one per island) over 3 iterations.

A total of 7 runs yield 21 programs.

### Program Generation with LLM + FunSearch
To evolve TSP-solving programs using a large language model and FunSearch, follow the steps below:

✅ Environment Setup
Use Python 3.11
Create and activate a virtual environment with Python 3.11:

python3.11 -m venv venv
source venv/bin/activate  # or venv\Scripts\activate on Windows
Install dependencies

pip install -r requirements.txt
⚙️ Run the FunSearch Evolution
Execute the evolution process using the following command:

python -m funsearch.run \
  ../examples/tsp_spec.py \
  2 \
  --sandbox_type ExternalProcessSandbox \
  --iterations 3
Explanation of arguments:

../examples/tsp_spec.py: the problem specification (TSP prompt and evaluation logic)

2: number of samplers (i.e., functions per prompt)

--iterations 3: number of evolution iterations

--sandbox_type ExternalProcessSandbox: isolates program execution for safety

### Best Program Selection & Evaluation
#### Select Top Programs
Run ranking_file.py to evaluate and select the best 7 programs based on their performance over 7 TSP instances (n = 20).

#### Evaluate Generalization
Run final_evaluation_generate.py to evaluate the 7 best programs on a broader set of TSP instances with 20, 50, and 100 cities.

#### Generate Summary Statistics
Run final_result_stats.py to compute:

Absolute and relative errors

Average performance

Program victories vs evaluators

### Outputs
The following result files are produced:

File	                    Description
test_table_{n_cities}.csv	Evaluation results for each program
ranking_resultado.csv	    Initial best-program selection ranking
victorias_detalle.csv	Per-instance LLM vs Evaluator winner info
conteo_victorias_por_identifier.csv	Victory count by program identifier
resumen_comparativo.csv	Mean, std, and max relative error summary

### License
This project is for academic research purposes.

