plugin_name  = "0"

def create(callback):
    return zero_gate()

class zero_gate:
    input_description = []
    output_description = [('output', int)]
    def run(self):
        return {'output': 0 }
