from svm_interface import LibsvmInputFile, LibsvmModelFile, TextFile, ConfigData
from svm_interface import svm_train_pathname
import util
import trainer_dialog
import os,tempfile

plugin_name = "Trainer"

def create(callback):
    return Trainer(callback)
    
class TrainerConfig:
    __slots__ = ['svm_type', 'kernel_type', 'cost', 'nu', 'gamma', 'degree',
                 'coef0', 'cache_size','shrinking', 'stopping_epsilon',
                 'regression_epsilon','fold' ]

class Trainer:
    input_description = [('training data', LibsvmInputFile),
                         ('optional parameter', ConfigData)]
    output_description = [('stdout/stderr', TextFile),
                          ('model', LibsvmModelFile)]
    def __init__(self,callback):        
        c = self.config = TrainerConfig()
        c.svm_type = 0
        c.kernel_type = 2
        c.cost = 1
        c.nu = 0.5
        c.gamma = 0
        c.degree = 3
        c.coef0 = 0
        c.cache_size = 40
        c.shrinking = 1
        c.stopping_epsilon = 0.001
        c.regression_epsilon = 0.1
        c.fold = 0
        self.callback = callback        
        
    def configure(self):        
        dlg = trainer_dialog.create(None,self.config)        
        try:
            return dlg.ShowModal()
        finally:
            dlg.Destroy()

    def run(self,**k):                   
        input = k['training data']
        if k['optional parameter']:
            self.config.__dict__.update(k['optional parameter'].get_data())
        filename = tempfile.mktemp()
        cmd = self.make_command(input.pathname, filename)
        log = self.callback.log
        log('# Running %s\n' % cmd)
        t = util.tee(log)
        util.run_command(cmd,t,t)
        
        if self.config.fold == 0:
            m = LibsvmModelFile(filename,autodel=True)
        else:
            m = None    # no model file while doing cv
            
        return {'stdout/stderr': TextFile(t.get_filename(), autodel=True),
                'model': m }

    def make_command(self,input_filename,output_filename):
        c = self.config

        # always
        _type = (c.svm_type != 0 and '-s %s ' % c.svm_type or '')
        _kernel = (c.kernel_type != 2 and '-t %s ' % c.kernel_type or '')
        _cache = (c.cache_size != 40 and '-m %s ' % c.cache_size or '')
        _stopping_epsilon = (c.stopping_epsilon != 0.001 and '-e %s ' % c.stopping_epsilon or '')
        _shrinking = (c.shrinking != 1 and '-h %s ' % c.shrinking or '')
        # optional
        _cost = (c.cost != 1 and '-c %s ' % c.cost or '')
        _nu = (c.nu != 0.5 and '-n %s ' % c.nu or '')
        _regression_epsilon = (c.regression_epsilon != 0.1 and '-p %s ' % c.regression_epsilon or '')
        _degree = (c.degree != 3 and '-d %s ' % c.degree or '')
        _gamma = (c.gamma != 0 and '-g %s ' % c.gamma or '')
        _coef0 = (c.coef0 != 0 and '-r %s ' % c.coef0 or '')
        _fold = (c.fold != 0 and '-v %s ' % c.fold or '')
        
        cmd = svm_train_pathname + ' '
        cmd += _type + _kernel + _cache + _stopping_epsilon + _shrinking
        
        if c.fold > 0: cmd += _fold
        
        svm_type = c.svm_type
        if svm_type == 0:
            cmd += _cost            
        elif svm_type == 1:
            cmd += _nu            
        elif svm_type == 2:
            cmd += _nu            
        elif svm_type == 3:
            cmd += _regression_epsilon + _cost
        elif svm_type == 4:
            cmd += _nu + _cost
            
        kernel_type = c.kernel_type
        if kernel_type == 0:
            pass            
        elif kernel_type == 1:
            cmd += _degree + _gamma + _coef0            
        elif kernel_type == 2:
            cmd += _gamma            
        elif kernel_type == 3:
            cmd += _gamma + _coef0            
        
        return cmd + '%s %s' % (input_filename, output_filename)
