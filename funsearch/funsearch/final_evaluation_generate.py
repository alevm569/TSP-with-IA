import os
import csv
import subprocess
import re
from pathlib import Path
from eval_tsp.plotUtil import read_solution_from_pickle

n_cities = 100
n_cities_nodes = 20
folder_path = fr"C:\Users\valer\Documents\Respaldo\USFQ\MAESTRIA_IA\Tesis\TSP-with-IA\funsearch\tests\tsp-{n_cities}"
programs_folder = fr"C:\Users\valer\Documents\Respaldo\USFQ\MAESTRIA_IA\Tesis\TSP-with-IA\funsearch\funsearch\data\best_programs_solutions"
ranking_csv_path = r"C:\Users\valer\Documents\Respaldo\USFQ\MAESTRIA_IA\Tesis\TSP-with-IA\funsearch\funsearch\ranking_resultado.csv"
output_folder = fr"C:\Users\valer\Documents\Respaldo\USFQ\MAESTRIA_IA\Tesis\TSP-with-IA\funsearch\funsearch\test_tables_info\test_final_tables"
output_csv = os.path.join(output_folder, f"test_table_{n_cities}.csv")


def run_program_with_matrix(program_path, matrix_code_str):
    program_path = Path(program_path)
    temp_path = program_path.parent / f"temp_{program_path.name}"

    with open(program_path, "r", encoding="utf-8") as f:
        content = f.read()

    # Verificar si ya existe import numpy
    if "import numpy as np" not in content:
        content = "import numpy as np\n" + content

    pattern = re.compile(r"^( *)matrix_distances = read_distance_matrix\(\)", re.MULTILINE)
    match = pattern.search(content)
    if not match:
        return "ERROR: No se encontró la línea matrix_distances = read_distance_matrix() en el programa"

    indent = match.group(1)
    replacement_line = f"{indent}matrix_distances = {matrix_code_str}\n{indent}matrix_distances = np.array(matrix_distances)"

    modified_code = pattern.sub(replacement_line, content)

    # Agregar prints para debugging
    modified_code += """
try:
    resultado = evaluate(0)
    print("DEBUG: Resultado de evaluate(0):", resultado)
except Exception as e:
    print("DEBUG: Error en evaluate:", e)
"""

    with open(temp_path, "w", encoding="utf-8") as f:
        f.write(modified_code)

    try:
        result = subprocess.run(
            ["python", str(temp_path)],
            capture_output=True,
            text=True,
            timeout=15
        )
        print(result.stdout)
        print(result.stderr)

        output_lines = result.stdout.strip().splitlines()
        for line in output_lines:
            if line.startswith("DEBUG: Resultado de evaluate(0):"):
                value_str = line.split(":", 2)[2].strip()
                try:
                    return float(value_str)
                except:
                    return f"ERROR: output no es float: {value_str}"
        return "ERROR: No se encontró la salida de evaluate(0) en stdout."
    except Exception as e:
        return f"ERROR: {str(e)}"
    finally:
        try:
            temp_path.unlink()
        except:
            pass


# Read programs
programs = []
with open(ranking_csv_path, newline='', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    for row in reader:
        programs.append({
            "identifier": row["Identifier"],
            "isla": row["Isla"]
        })
        if len(programs) == 7:
            break

files = sorted([f for f in os.listdir(folder_path) if f.endswith(".pkl")])[:20]
os.makedirs(output_folder, exist_ok=True)

with open(output_csv, mode="w", newline="", encoding="utf-8") as fcsv:
    writer = csv.writer(fcsv)
    writer.writerow([
        "File",
        "identifier",
        "isla",
        "distancia LP-ACO",
        "distancia LLM",
        "error absoluto",
        "error relativo"
    ])

    for file in files:
        file_path = os.path.join(folder_path, file)
        tsp_solution = read_solution_from_pickle(file_path)
        distancia_lpaco = tsp_solution.distance

        matrix_distances_code = repr(tsp_solution.matrix_distances.tolist())

        for prog in programs:
            identifier = prog["identifier"]
            isla = prog["isla"]
            program_filename = f"best_program_{identifier}_{isla}_{n_cities_nodes}.py"
            program_path = os.path.join(programs_folder, program_filename)

            distancia_llm = run_program_with_matrix(program_path, matrix_distances_code)

            if isinstance(distancia_llm, float):
                error_abs = abs(distancia_llm - distancia_lpaco)
                error_rel = error_abs / distancia_lpaco if distancia_lpaco != 0 else 0
            else:
                error_abs = "N/A"
                error_rel = "N/A"

            writer.writerow([
                file.replace(".pkl", ""),
                identifier,
                isla,
                distancia_lpaco,
                distancia_llm,
                error_abs,
                error_rel
            ])

            print(f"✅ {file} | {identifier} | LP-ACO: {distancia_lpaco:.6f} | LLM: {distancia_llm}")

print(f"\n✅ Archivo CSV generado en: {output_csv}")
