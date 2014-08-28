from svm_interface import LibsvmInputFile, TextFile, ImageFile, ConfigData
from svm_interface import svm_train_pathname, grid_pathname, gnuplot_pathname
import util
import tempfile
import grid_dialog

plugin_name = "Grid"

def create(callback):
    return Grid(callback)

class GridConfig:
    __slots__ = ['c_begin', 'c_end', 'c_step',
                 'g_begin', 'g_end', 'g_step',
                 'fold']
    
class Grid:
    input_description = [('training data', LibsvmInputFile)]
    output_description = [('contour',ImageFile),
                          ('result',TextFile),
                          ('stdout/stderr', TextFile),
                          ('best parameter', ConfigData)]

    def __init__(self,callback):
        self.callback = callback
        c = self.config = GridConfig()        
        c.c_begin, c.c_end, c.c_step = -5,  15, 2
        c.g_begin, c.g_end, c.g_step =  3, -15, -2
        c.fold = 5

    def configure(self):
        dlg = grid_dialog.create(None,self.config)
        try:
            return dlg.ShowModal()
        finally:
            dlg.Destroy()
            
    def run(self,**k):
        input = k['training data']
        out_filename = tempfile.mktemp()
        png_filename = tempfile.mktemp()
        c = self.config
        cmd = "%s -svmtrain %s " % (grid_pathname, svm_train_pathname)
        cmd += "-gnuplot %s -out %s " % (gnuplot_pathname, out_filename)
        cmd += "-png %s " % png_filename
        cmd += "-log2c %s,%s,%s " % (c.c_begin, c.c_end, c.c_step)
        cmd += "-log2g %s,%s,%s " % (c.g_begin, c.g_end, c.g_step)
        cmd += input.pathname
        log = self.callback.log
        log('# Running %s\n' % cmd) 
        t = util.tee(log)
        util.run_command(cmd,t,t)
        best_c, best_g, best_rate = map(float,open(t.get_filename()).readlines()[-1].split())
        return {'contour': ImageFile(png_filename,autodel=True),
                'result': TextFile(out_filename,autodel=True),
                'stdout/stderr': TextFile(t.get_filename(),autodel=True),
                'best parameter': ConfigData(cost=best_c,gamma=best_g)} 
