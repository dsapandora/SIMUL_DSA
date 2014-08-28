import os
import ViewImageFrame
import ViewTextFrame

class File(object): 
    def __init__(self,pathname,autodel=False):
        self.pathname = pathname
        self.autodel = autodel
    def __del__(self):
        if self.autodel:            
            os.remove(self.pathname)
    def view(self,parent,title):
        f = ViewTextFrame.create(parent)            
        f.SetTitle(title)
        f.set_text(open(self.pathname).read())       
        f.Show()

class TextFile(File):
    pass

class ImageFile(File):
    def view(self,parent,title):
        f = ViewImageFrame.create(parent)
        f.SetTitle(title)
        f.set_image(self.pathname)
        f.Show()        
