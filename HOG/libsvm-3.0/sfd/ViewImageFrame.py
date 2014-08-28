#Boa:Frame:ViewImageFrame

from wxPython.wx import *

def create(parent):
    return ViewImageFrame(parent)

[wxID_VIEWIMAGEFRAME] = map(lambda _init_ctrls: wxNewId(), range(1))

class ViewImageFrame(wxFrame):
    def _init_utils(self):
        # generated method, don't edit
        pass

    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wxFrame.__init__(self, id=wxID_VIEWIMAGEFRAME, name='ViewImageFrame',
              parent=prnt, pos=wxPoint(190, 205), size=wxSize(408, 227),
              style=wxFRAME_FLOAT_ON_PARENT | wxDEFAULT_FRAME_STYLE,
              title='ViewImageFrame')
        self._init_utils()
        self.SetClientSize(wxSize(400, 200))
        self.SetBackgroundColour(wxColour(255, 255, 255))

    def __init__(self, parent):
        self._init_ctrls(parent)

    def set_image(self, filename):
        img = wxImage(filename).ConvertToBitmap()
        size = wxSize(img.GetWidth(), img.GetHeight())
        self.SetClientSize(size)
        wxStaticBitmap(self, -1, img, wxPoint(0,0),size)
                       
