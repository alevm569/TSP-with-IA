from StatsProblemManager2 import StatsProblemManager2

manager = StatsProblemManager2()

# Simular dos islas
isla_1 = manager.get_stats_by_island("isla-1")
isla_2 = manager.get_stats_by_island("isla-2")

# Registrar stats en cada una
isla_1.register_stats(100, 500, 1)
isla_2.register_stats(90, 400, 1)

# Volver a accederlas
otra_vez_isla_1 = manager.get_stats_by_island("isla-1")
print("\n🔁 ¿Se mantiene el estado de isla-1?")
print("Resultado guardado:", otra_vez_isla_1.last_best_result)  # Debe ser 100

# Probar el best_solution global
manager.best_solution.register_stats(80, 300, 1)
print("\n🌍 Global best solution:", manager.best_solution.last_best_result)  # Debe ser 80
