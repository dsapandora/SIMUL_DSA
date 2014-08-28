import tempfile
import threading
import os
import sys,traceback

def get_traceback():
    return ''.join(traceback.format_exception(*sys.exc_info()))

class curry:
    def __init__(self,f,*args,**kwargs):
        self.f = f
        self.args = args
        self.kwargs = kwargs
    def __call__(self,*args,**k):
        for key,value in self.kwargs.items():
            if key not in k:
                k[key] = value
        return self.f(*(self.args+args), **k)

from wxPython.wx import *
def MsgBox(*args):
    dlg = wxMessageDialog(None, ','.join(map(str,args)),
      'MsgBox', wxOK | wxICON_INFORMATION)
    try:
        dlg.ShowModal()
    finally:
        dlg.Destroy()    

def GetInput(question="Input:", caption='', default=''):
    answer = ""
    dlg = wxTextEntryDialog(None, question, caption, default)    
    try:
        if dlg.ShowModal() == wxID_OK:
            answer = dlg.GetValue()            
    finally:
        dlg.Destroy()
    return answer    

def GetFilename(default_file=''):
    filename = ""
    dlg = wxFileDialog(None, "Choose a file", ".", default_file, "*", wxOPEN)
    try:
        if dlg.ShowModal() == wxID_OK:
            filename = dlg.GetPath()
    finally:
        dlg.Destroy()
    return filename

class tee:
    def __init__(self,next=None):
        self.filename = tempfile.mktemp()
        self.f = open(self.filename,'wb')
        self.next = next
    def __call__(self,data):
        self.f.write(data)
        if self.next:
            self.next(data)
    def get_filename(self):
        self.f.close()
        return self.filename
        
def run_command(cmd,log_stdout=None,log_stderr=None):            
    if log_stderr is None:
        if log_stdout is None:
            os.system(cmd)
        else:            
            inp, outp = os.popen2(cmd)
            while 1:
                s = outp.read()
                if not s: break
                log_stdout(s)
            inp.close()
            outp.close()
    else:
        if log_stdout is None:
            inp, outp, errp = os.popen3(cmd)
            while 1:
                s = errp.read()
                if not s: break
                log_stderr(s)
            inp.close()
            outp.close()
            errp.close()
        elif log_stdout is log_stderr:
            inp, outerrp = os.popen4(cmd)
            while 1:
                s = outerrp.read()
                if not s: break
                log_stdout(s)
            inp.close()
            outerrp.close()
        else:
            run_command_hard(cmd,log_stdout,log_stderr)

def run_command_hard(cmd,log_stdout,log_stderr):    
    inp, outp, errp = os.popen3(cmd)    
    def out_loop():
        while 1:
            s = outp.read()
            if not s: break
            log_stdout(s)
        outp.close()
    def err_loop():
        while 1:
            s = errp.read()
            if not s: break
            log_stderr(s)
        errp.close()
    t1 = threading.Thread(target=out_loop)
    t2 = threading.Thread(target=err_loop)
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    inp.close()

    
