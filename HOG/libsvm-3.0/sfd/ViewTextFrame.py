#Boa:Frame:ViewTextFrame

from wxPython.wx import *

def create(parent):
    return ViewTextFrame(parent)

[wxID_VIEWTEXTFRAME, wxID_VIEWTEXTFRAMETEXTCTRL1, 
] = map(lambda _init_ctrls: wxNewId(), range(2))

class ViewTextFrame(wxFrame):
    def _init_utils(self):
        # generated method, don't edit
        pass

    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wxFrame.__init__(self, id=wxID_VIEWTEXTFRAME, name='ViewTextFrame',
              parent=prnt, pos=wxPoint(320, 349), size=wxSize(400, 200),
              style=wxFRAME_FLOAT_ON_PARENT | wxDEFAULT_FRAME_STYLE,
              title='ViewTextFrame')
        self._init_utils()
        self.SetClientSize(wxSize(400, 200))

        self.textCtrl1 = wxTextCtrl(id=wxID_VIEWTEXTFRAMETEXTCTRL1,
              name='textCtrl1', parent=self, pos=wxPoint(0, 0), size=wxSize(400,
              200),
              style=wxTE_READONLY | wxHSCROLL | wxVSCROLL | wxTE_MULTILINE | wxTE_RICH,
              value='textCtrl1')

    def __init__(self, parent):
        self._init_ctrls(parent)
        self.textCtrl1.SetFont(wxFont(12, wxMODERN, wxNORMAL, wxNORMAL))

    def set_text(self,text):
        self.textCtrl1.Clear()
        self.textCtrl1.AppendText(text)
