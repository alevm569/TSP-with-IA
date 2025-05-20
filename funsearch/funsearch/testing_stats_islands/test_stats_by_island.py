from StatsByIsland import StatsByIsland

# Instanciamos una nueva isla
isla = StatsByIsland()

# Primera solución (se debe registrar)
print("\n🔹 Primera solución")
isla.register_stats(100, 500, 1)
print("last_best_result:", isla.last_best_result)
print("elapsed_time_ms:", isla.elapsed_time_ms)
print("iteration_number:", isla.iteration_number)
print("patience:", isla.patience)

# Solución peor, pero con mejor tiempo (solo actualiza tiempo)
print("\n🔹 Solución igual, mejor tiempo")
isla.register_stats(100, 400, 2)
print("last_best_result:", isla.last_best_result)
print("elapsed_time_ms:", isla.elapsed_time_ms)
print("iteration_number:", isla.iteration_number)
print("patience:", isla.patience)

# Solución peor y tiempo peor (debería aumentar paciencia)
print("\n🔹 Solución peor, tiempo peor")
isla.register_stats(110, 600, 3)
print("last_best_result:", isla.last_best_result)
print("elapsed_time_ms:", isla.elapsed_time_ms)
print("iteration_number:", isla.iteration_number)
print("patience:", isla.patience)

# Solución mejor (debería reiniciar paciencia y actualizar todo)
print("\n🔹 Solución mejor")
isla.register_stats(90, 300, 4)
print("last_best_result:", isla.last_best_result)
print("elapsed_time_ms:", isla.elapsed_time_ms)
print("iteration_number:", isla.iteration_number)
print("patience:", isla.patience)
