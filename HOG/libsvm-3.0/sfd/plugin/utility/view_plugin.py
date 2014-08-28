from common_interface import File, ImageFile
import ViewTextFrame, ViewImageFrame

plugin_name  = "Viewer"

def create(callback):
    return Viewer(callback)

def show(input):
    if isinstance(input, ImageFile):
        f = ViewImageFrame.create(None)
        f.set_image(input.pathname)
        f.Show()
    else:
        f = ViewTextFrame.create(None)
        f.set_text(open(input.pathname).read())
        f.Show()

class Viewer:
    input_description = [('input', File)]
    output_description = [('output', None)]

    def __init__(self,callback):
        self.callback = callback

    def run(self,input):
        self.callback.run(lambda: show(input))
        return {'output': input}
