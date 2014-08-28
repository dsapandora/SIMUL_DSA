from svm_interface import LibsvmInputFile, LibsvmModelFile, TextFile
from svm_interface import svm_predict_pathname
import util
import tempfile

plugin_name = "Predictor"

def create(callback):
    return Predictor(callback)

class Predictor:
    input_description = [('test data', LibsvmInputFile),
                         ('model', LibsvmModelFile)]
    output_description = [('output', TextFile)]
    def __init__(self,callback):
        self.callback = callback
    def run(self,**k):
        input = k['test data']
        model = k['model']
        filename = tempfile.mktemp()                        
        cmd = "%s %s %s %s" % (svm_predict_pathname, input.pathname,
                               model.pathname, filename)
        log = self.callback.log
        log('# Running %s\n' % cmd) 
        util.run_command(cmd,log,log)
        return {'output': TextFile(filename,autodel=True) }
