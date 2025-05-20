class FakeFunction:
    def __init__(self):
        self.body = ""

class FakeProgram:
    def __init__(self):
        self._function = FakeFunction()

    def get_function(self, name):
        return self._function

    def __str__(self):
        return f"Program<{self._function.body}>"
