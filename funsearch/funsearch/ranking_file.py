import csv
import os
from pathlib import Path

# Paths
INPUT_DIR = Path("test_tables_info")
OUTPUT_FILE = "ranking_resultado.csv"
N_CITIES = 20

ranking_data = []


def clean_distance(valor):
    """Convierte string tipo '19,77' a float 19.77"""
    try:
        return float(valor.replace(",", "."))
    except:
        return float('inf')


def find_nearest(distancia_eval, filas):
    """Busca la fila cuya distancia sea más cercana a distancia_eval"""
    return min(filas, key=lambda fila: abs(clean_distance(fila[3]) - distancia_eval))


# Repetir el proceso 7 veces
for i in range(7):
    print(f"\n Entrada {i + 1}/7")

    file_hash = input("👉 Ingresa el file_hash: ").strip()
    distancia_evaluador = float(input("Ingresa la distancia del evaluador: ").strip())

    input_csv_path = INPUT_DIR / f"{file_hash}.csv"

    if not input_csv_path.exists():
        print(f"No se encontró el archivo: {input_csv_path}")
        continue

    with open(input_csv_path, newline='', encoding='utf-8') as csvfile:
        reader = csv.reader(csvfile)
        next(reader) 
        filas = [row for row in reader]

    fila_cercana = find_nearest(distancia_evaluador, filas)

    # Add to ranking
    ranking_data.append([
        file_hash,  # File
        fila_cercana[1],  # Identifier
        N_CITIES,  # n_cities
        fila_cercana[2],  # Isla
        distancia_evaluador,  # Distancia evaluador
        clean_distance(fila_cercana[3])  # Distancia LLM
    ])

# Save
with open(OUTPUT_FILE, "w", newline='', encoding='utf-8') as out_csv:
    writer = csv.writer(out_csv)
    writer.writerow(["File", "Identifier", "n_cities", "Isla", "distance evaluador", "distance llm"])
    writer.writerows(ranking_data)

print(f"\n✅ ¡Proceso completo! Archivo guardado en: {OUTPUT_FILE}")
