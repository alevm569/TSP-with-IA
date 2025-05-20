class FakeSandbox:
    def __init__(self, output_map):
        # output_map: dict[input, output]
        self.output_map = output_map

    def run(self, program_str, function_name, current_input, timeout):
        output = self.output_map.get(current_input, 999)
        return output, True  # Siempre corre OK
