from svm_interface import LibsvmInputFile, LibsvmScaleFile, svm_scale_pathname
import scaler_dialog
import util
import tempfile

plugin_name = "Scaler"

def create(callback):
    return Scaler(callback)

class ScalerConfig:
    __slots__ = ['lower','upper','y_lower','y_upper']
    
class Scaler:
    input_description = [('input', LibsvmInputFile),
                         ('scale info input', LibsvmScaleFile)]
    output_description = [('output', LibsvmInputFile),
                          ('scale info output', LibsvmScaleFile)]
    def __init__(self,callback):
        self.callback = callback
        c = self.config = ScalerConfig()
        c.lower = -1        
        c.upper = 1
        c.y_lower = None
        c.y_upper = None
    def run(self,input,**k):
        c = self.config
        cmd = '%s -l %g -u %g ' % (svm_scale_pathname, c.lower, c.upper)
        if c.y_lower:
            cmd += "-y %g %g " % (c.y_lower, c.y_upper)
        if k['scale info input']:
            cmd += '-r %s ' % k['scale info input'].pathname
        filename = tempfile.mktemp()
        sio = LibsvmScaleFile(filename,autodel=True)
        cmd += '-s %s ' % filename
        cmd += input.pathname        
        log = self.callback.log
        log('# Running %s\n' % cmd) 
        t = util.tee()
        util.run_command(cmd,t,log)
        return {'output': LibsvmInputFile(t.get_filename(),autodel=True),
                'scale info output': sio}
    def configure(self):
        dlg = scaler_dialog.create(None,self.config)
        try:
            return dlg.ShowModal()
        finally:
            dlg.Destroy()
