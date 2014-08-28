from svm_interface import LibsvmInputFile
from random import randint
from util import GetInput
import os,tempfile

plugin_name = "Random Sampler"

def create(callback):
    return Sampler()
    
class Sampler:
    input_description = [('input', LibsvmInputFile)]
    output_description = [('output', LibsvmInputFile)]
    def __init__(self):
        self.n = 100
        self.configure()
    def configure(self):
        n = GetInput("Number of samples to choose:", default=str(self.n))
        if n:
            self.n = int(n)
            return True
    def run(self,input):
        lines = open(input.pathname).readlines()
        filename = tempfile.mktemp()
        f = open(filename,'w')
        l = len(lines)
        n = int(self.n)
        for i in range(l):
        	if randint(0,l-i-1) < n:
        		f.write(lines[i])
        		n = n-1
        f.close()
        return {'output' : LibsvmInputFile(filename,autodel=True)}
