from util import GetFilename
from common_interface import File
import shutil

plugin_name  = "Save to File"

def create(callback):
    filename = GetFilename()
    if not filename: return None
    return SaveToFile(filename)

class SaveToFile:
    input_description = [('input', File)]
    output_description = [('output', None)]
    def __init__(self,filename):
        self.filename = filename
    def configure(self):
        filename = GetFilename(self.filename)
        if filename: 
            self.filename = filename
            return True
    def run(self,input):
        shutil.copyfile(input.pathname, self.filename)
        return {'output': input}
