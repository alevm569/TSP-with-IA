import requests
import json

model_name_default = "codegemma:latest"
api_endpoint_default = 'http://172.21.230.10:11444/api/generate'


class OLLAMA:
    def __init__(self, model_name=model_name_default, api_endpoint=api_endpoint_default, **kwargs):
        self.model_name = model_name
        self.api_endpoint = api_endpoint
        self.session = requests.Session()
        self.kwargs = {"temperature": 0.7, "n": 1, **kwargs}

        print(f"Initialized OLLAMA with model_name: {model_name}, api_endpoint: {api_endpoint}, kwargs: {self.kwargs}")

    def predict(self, question, **kwargs):
        output = ""
        payload = {'model': self.model_name, 'prompt': question, **self.kwargs, **kwargs}

        # Use the session to post the request with the payload
        with self.session.post(self.api_endpoint, json=payload, stream=True) as r:
            # Ensure the response status is OK
            if r.status_code == 200:
                for line in r.iter_lines():
                    # Decode each line that is not empty
                    if line:
                        j = json.loads(line.decode('utf-8'))  # Ensure decoding from bytes to string
                        output += j.get("response", "")
                        # Break if the 'done' flag is True
                        if j.get("done", True):
                            break
            else:
                print(f"Error: Received status code {r.status_code}")

        return [output.strip()]

    def prompt(self, question, **kwargs):
        """Alias para compatibilidad con el resto del código."""
        return self.predict(question, **kwargs)
    def __call__(self, question, **kwargs):
        return self.predict(question, **kwargs)
