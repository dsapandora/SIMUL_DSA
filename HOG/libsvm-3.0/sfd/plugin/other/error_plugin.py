plugin_name  = "Error"

def create(callback):
    return error()

class error:
    input_description = [('input', object)]
    output_description = []
    def run(self,input):
        raise Exception,"Error Plugin"
