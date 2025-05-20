from funsearch.StatsByIsland import StatsByIsland

s = StatsByIsland()
s.set_stats_path("stats_per_island/stats_island_test.json")
s.register_stats(100, 300, 1)
