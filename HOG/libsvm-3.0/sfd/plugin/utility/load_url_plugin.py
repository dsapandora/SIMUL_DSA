from util import GetInput
from common_interface import File
import urllib

plugin_name  = "Load File from URL"

def create(callback):
    url = GetInput(question="URL:")
    if not url: return None
    return LoadURL(url)

class LoadURL:
    input_description = []
    output_description = [('output', File)]
    def __init__(self,url):
        self.url = url
    def configure(self):
        url = GetInput(question="URL:",default=self.url)
        if url: 
            self.url = url
            return True
    def run(self):
        filename, info = urllib.urlretrieve(self.url)
        return {'output': File(filename)}
