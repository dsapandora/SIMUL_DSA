plugin_name  = "1"

def create(callback):
    return one_gate()

class one_gate:
    input_description = []
    output_description = [('output', int)]
    def run(self):
        return {'output': 1 }
