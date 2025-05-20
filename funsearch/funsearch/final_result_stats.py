import os
import pandas as pd

# Path
csv_dir = r"C:\Users\valer\Documents\Respaldo\USFQ\MAESTRIA_IA\Tesis\TSP-with-IA\funsearch\funsearch\test_tables_info\test_final_tables"

# Files
csv_files = [f for f in os.listdir(csv_dir) if f.endswith(".csv")]

# Save results
stats_comparativos = []
victorias_globales = []

for filename in csv_files:
    file_path = os.path.join(csv_dir, filename)
    df = pd.read_csv(file_path)

    df["media"] = df[["distancia LP-ACO", "distancia LLM"]].mean(axis=1)

    df["ganador"] = df.apply(
        lambda row: "Empate" if row["distancia LLM"] == row["distancia LP-ACO"]
        else "LLM" if row["distancia LLM"] < row["distancia LP-ACO"]
        else "Evaluador",
        axis=1
    )

    victorias_globales.append(df[[
        "File", "identifier", "isla", "distancia LP-ACO", "distancia LLM", "media", "ganador"
    ]])

    # Global stats
    stats = {
        "archivo": filename,
        "mean_rel_error": df["error relativo"].mean(),
        "std_rel_error": df["error relativo"].std(),
        "max_rel_error": df["error relativo"].max()
    }
    stats_comparativos.append(stats)

# Final DataFrames
comparativos_df = pd.DataFrame(stats_comparativos)
victorias_df = pd.concat(victorias_globales, ignore_index=True)

# Win count by identifier (includes ties)
conteo_victorias = victorias_df.groupby(["identifier", "ganador"]).size().unstack(fill_value=0).reset_index()

# Calculate means separated by identifier
media_por_identifier = victorias_df.groupby("identifier")[["distancia LP-ACO", "distancia LLM"]].mean().reset_index()
media_por_identifier.rename(columns={
    "distancia LP-ACO": "media_LP_ACO",
    "distancia LLM": "media_LLM"
}, inplace=True)


# Save
comparativos_df.to_csv("resumen_comparativo.csv", index=False)
victorias_df.to_csv("victorias_detalle.csv", index=False)
conteo_victorias.to_csv("conteo_victorias_por_identifier.csv", index=False)
media_por_identifier.to_csv("media_por_identifier.csv", index=False)



# Mostrar resumen
print("Resumen general por archivo:")
print(comparativos_df)

print("\nEjemplo de resultados de victorias:")
print(victorias_df.head())

print("\nConteo de victorias por identifier:")
print(conteo_victorias)
