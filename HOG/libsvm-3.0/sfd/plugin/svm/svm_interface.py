import os,sys
from common_interface import *

class LibsvmInputFile(TextFile):
    pass

class LibsvmModelFile(TextFile):
    pass

class LibsvmScaleFile(TextFile):
    pass
    
class ConfigData(object):
    def __init__(self,**kw):
        self.data = kw
    def get_data(self):
        return self.data
    def __repr__(self):
        return 'ConfigData:'+repr(self.data)

# libsvm executable path

libsvm_path = "~/libsvm-2.4"
gnuplot_pathname = "/usr/bin/gnuplot"
join = os.path.join
if sys.platform == 'win32':
    svm_scale_pathname = join(libsvm_path,r'windows\svmscale.exe')
    svm_train_pathname = join(libsvm_path,r'windows\svmtrain.exe')
    svm_predict_pathname = join(libsvm_path,r'windows\svmpredict.exe')
    grid_pathname = join(libsvm_path,r'python\grid.py')
else:
    svm_scale_pathname = join(libsvm_path,'svm-scale')
    svm_train_pathname = join(libsvm_path,'svm-train')
    svm_predict_pathname = join(libsvm_path,'svm-predict')
    grid_pathname = join(libsvm_path,'python/grid.py')
