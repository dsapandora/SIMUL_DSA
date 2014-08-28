plugin_name  = "NOT gate"

def create(callback):
    return not_gate()

class not_gate:
    input_description = [('input', int)]
    output_description = [('output', int)]
    def run(self,input):
        return {'output': not input}
