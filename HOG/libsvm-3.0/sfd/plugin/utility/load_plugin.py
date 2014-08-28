from util import GetFilename
from common_interface import File

plugin_name  = "Load File"

def create(callback):
    filename = GetFilename()
    if not filename: return None
    return LoadFromFile(filename)

class LoadFromFile:
    input_description = []
    output_description = [('output', File)]
    def __init__(self,filename):
        self.filename = filename
    def configure(self):
        filename = GetFilename(self.filename)
        if filename: 
            self.filename = filename
            return True
    def run(self):
        return {'output': File(self.filename)}
