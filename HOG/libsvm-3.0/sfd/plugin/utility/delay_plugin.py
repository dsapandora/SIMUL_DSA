plugin_name  = "Delay"
import util
import time

def create(callback):
    n = util.GetInput(question="seconds:",default="2")
    if not n: return
    return delay(n)

class delay:
    input_description = [('input', object)]
    output_description = [('output', None)]
    def __init__(self,n):
        self.n = n
    def configure(self):
        n = util.GetInput(question="seconds:",default=self.n)
        if not n: return
        self.n = n
        return True
    def run(self,input):
        time.sleep(int(self.n))
        return {'output': input}
