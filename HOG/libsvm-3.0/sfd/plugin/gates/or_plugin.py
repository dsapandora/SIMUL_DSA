plugin_name  = "OR gate"

def create(callback):
    return or_gate()

class or_gate:
    input_description = [('input 1', int),('input 2', int)]
    output_description = [('output', int)]
    def run(self,**k):
        val = (k['input 1'] or k['input 2'])
        return {'output': val }
