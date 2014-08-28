from util import GetInput
from common_interface import File
import util,tempfile

plugin_name  = "General Command"

def create(callback):
    command = GetInput(question='Command:')
    if not command: return None
    return Piper(command,callback)

class Piper:
    input_description = [('input', File)]
    output_description = [('output', None)]
    def __init__(self,command,callback):
        self.command = command
        self.callback = callback
    def configure(self):
        command = GetInput(question='Command:',default=self.command)
        if command: 
            self.command = command
            return True
    def run(self,input):
        filename = tempfile.mktemp()
        f = open(filename,'wb')
        cmd = self.command
        if input: cmd += '< %s' % input.pathname
        util.run_command(cmd, f.write, self.callback.log)
        f.close()
        return {'output': File(filename)}
