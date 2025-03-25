import re


class CodeExtractor:
    def __init__(self, code: str, function_name: str):
        self.code = code
        self.function_name = function_name

    def extract(self):
        last_version_name = self.last_version_function_name()
        function_code = self.capture_function_implementation(last_version_name)
        return function_code, last_version_name

    def extract_function_names(self):
        regex = r"def (.*?)\(.*?\) -> .*:"
        matches = re.findall(regex, self.code)
        return matches

    def last_version_function_name(self):
        function_names = self.extract_function_names()
        filter_functions = [f for f in function_names if self.function_name in f]
        # get number of version
        version = 0
        last_version_name = None
        for f in filter_functions:
            regex = r"\d+"
            match = re.search(regex, f)
            if match:
                if int(match.group(0)) > version:
                    last_version_name = f
        return last_version_name

    def capture_function_implementation(self, function_name):
        # use function name to get just the code of teh function
        regex = rf"def {function_name}\(.*?\) -> .*:(?:\s*(?:[ \t]*(?!def|#|`|').*(?:\n|$)))+"
        match = re.search(regex, self.code, re.DOTALL)
        if match:
            return match.group(0)
        return None