import os
import subprocess
import csv
import re
from pathlib import Path

output_dir = os.path.join(os.getcwd(), "test_tables_info")
os.makedirs(output_dir, exist_ok=True)
# CONFIG MANUAL
n_file = 11
file_hash = "b6bb0fa521c532065da8243023bffc48"

identifiers = [
    "1746499826",
    "1746490919",
    "1746492541",
    "1746493335",
    "1746494622",
    "1746496504",
    "1746498171"
]

# Paths base
PROGRAMS_PATH = Path(
    "C:/Users/valer/Documents/Respaldo/USFQ/MAESTRIA_IA/Tesis/TSP-with-IA/funsearch/funsearch/data/best_programs_solutions")
output_csv = os.path.join(output_dir, f"{file_hash}.csv")


def extract_info_from_filename(filename):
    match = re.match(r"best_program_(\d+)_(\d+)_(\d+)\.py", filename)
    if match:
        return match.groups()  # identifier, isla, n_cities
    return None


results = []

# Loop through all identifiers
for identifier in identifiers:
    all_programs = [
        p for p in PROGRAMS_PATH.glob(f"best_program_{identifier}_*.py")
        if "None" not in p.name
    ]

    for program_path in all_programs:
        fname = program_path.name
        info = extract_info_from_filename(fname)
        if not info:
            continue
        prog_identifier, isla, _ = info

        # Create a temporary copy of the file by adding evaluate
        with open(program_path, "r", encoding="utf-8") as f:
            content = f.read()

        temp_path = program_path.parent / f"temp_{fname}"
        with open(temp_path, "w", encoding="utf-8") as f:
            f.write(content.strip() + "\nprint(evaluate(0))\n")

        # Run the file and capture the output
        try:
            result = subprocess.run(
                ["python", str(temp_path)],
                capture_output=True,
                text=True,
                timeout=15
            )
            output_lines = result.stdout.strip().splitlines()
            distance = output_lines[-1].replace(".", ",") if output_lines else "NO OUTPUT"
        except Exception as e:
            distance = f"ERROR: {str(e)}"

        results.append((file_hash, prog_identifier, isla, distance))
        temp_path.unlink()

# Save
with open(output_csv, "w", newline='', encoding='utf-8') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["File", "Identifier", "Isla", "Distancia"])
    writer.writerows(results)

print(f"✅ CSV completo")
