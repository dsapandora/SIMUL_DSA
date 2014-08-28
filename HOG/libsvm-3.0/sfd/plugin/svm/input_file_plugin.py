from svm_interface import LibsvmInputFile
from util import GetFilename

plugin_name = "Input File"

def create(callback):
    filename = GetFilename()
    if not filename: return None
    return InputFile(filename)
    
class InputFile:
    input_description = []
    output_description = [('file', LibsvmInputFile)]
    def __init__(self,filename):
        self.filename = filename
    def configure(self):
        filename = GetFilename(self.filename)
        if filename: 
            self.filename = filename
            return True
    def run(self):        
        return {'file': LibsvmInputFile(self.filename) }
