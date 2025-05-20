class FakeProgramsDatabase:
    def __init__(self):
        self._programs = {}       # Último programa por isla
        self._best_programs = []  # Programas guardados como "mejores"

    def register_program(self, func_obj, island_id, results):
        self._programs[str(island_id)] = f"Program_{func_obj.body}"

    def get_last_program_by_island(self, island_id):
        return self._programs.get(str(island_id))

    def save_best_programs(self, program_str, score, island_id):
        print(f"🌟 Saved best program from island {island_id}: score={score}")
        self._best_programs.append((island_id, score, program_str))
