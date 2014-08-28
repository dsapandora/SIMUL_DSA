plugin_name  = "AND gate"

def create(callback):
    return and_gate()

class and_gate:
    input_description = [('input 1', int),('input 2', int)]
    output_description = [('output', int)]
    def run(self,**k):
        val = (k['input 1'] and k['input 2'])
        return {'output': val }
