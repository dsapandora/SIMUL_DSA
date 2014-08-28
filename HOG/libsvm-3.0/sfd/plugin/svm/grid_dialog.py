#Boa:Dialog:GridDialog

from wxPython.wx import *

def create(parent,config):
    return GridDialog(parent,config)

[wxID_GRIDDIALOG, wxID_GRIDDIALOGBUTTON_CANCEL, wxID_GRIDDIALOGBUTTON_OK, 
 wxID_GRIDDIALOGSTATICTEXT1, wxID_GRIDDIALOGSTATICTEXT2, 
 wxID_GRIDDIALOGSTATICTEXT3, wxID_GRIDDIALOGSTATICTEXT4, 
 wxID_GRIDDIALOGSTATICTEXT5, wxID_GRIDDIALOGSTATICTEXT6, 
 wxID_GRIDDIALOGSTATICTEXT7, wxID_GRIDDIALOGTEXT_C_BEGIN, 
 wxID_GRIDDIALOGTEXT_C_END, wxID_GRIDDIALOGTEXT_C_STEP, 
 wxID_GRIDDIALOGTEXT_FOLD, wxID_GRIDDIALOGTEXT_G_BEGIN, 
 wxID_GRIDDIALOGTEXT_G_END, wxID_GRIDDIALOGTEXT_G_STEP, 
] = map(lambda _init_ctrls: wxNewId(), range(17))

class GridDialog(wxDialog):
    def _init_utils(self):
        # generated method, don't edit
        pass

    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wxDialog.__init__(self, id=wxID_GRIDDIALOG, name='GridDialog',
              parent=prnt, pos=wxPoint(233, 230), size=wxSize(313, 163),
              style=wxDEFAULT_DIALOG_STYLE, title='Grid')
        self._init_utils()
        self.SetClientSize(wxSize(305, 136))

        self.staticText1 = wxStaticText(id=wxID_GRIDDIALOGSTATICTEXT1,
              label='log2c:', name='staticText1', parent=self, pos=wxPoint(16,
              20), size=wxSize(39, 16), style=0)
        self.staticText1.SetFont(wxFont(12, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.text_c_begin = wxTextCtrl(id=wxID_GRIDDIALOGTEXT_C_BEGIN,
              name='text_c_begin', parent=self, pos=wxPoint(64, 18),
              size=wxSize(48, 22), style=0, value='')
        self.text_c_begin.SetFont(wxFont(10, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.staticText2 = wxStaticText(id=wxID_GRIDDIALOGSTATICTEXT2,
              label='to', name='staticText2', parent=self, pos=wxPoint(120, 20),
              size=wxSize(12, 16), style=0)
        self.staticText2.SetFont(wxFont(12, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.text_c_end = wxTextCtrl(id=wxID_GRIDDIALOGTEXT_C_END,
              name='text_c_end', parent=self, pos=wxPoint(144, 18),
              size=wxSize(48, 22), style=0, value='')
        self.text_c_end.SetFont(wxFont(10, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.staticText3 = wxStaticText(id=wxID_GRIDDIALOGSTATICTEXT3,
              label='log2g:', name='staticText3', parent=self, pos=wxPoint(16,
              56), size=wxSize(40, 16), style=0)
        self.staticText3.SetFont(wxFont(12, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.text_g_begin = wxTextCtrl(id=wxID_GRIDDIALOGTEXT_G_BEGIN,
              name='text_g_begin', parent=self, pos=wxPoint(64, 56),
              size=wxSize(48, 22), style=0, value='')
        self.text_g_begin.SetFont(wxFont(10, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.staticText4 = wxStaticText(id=wxID_GRIDDIALOGSTATICTEXT4,
              label='to', name='staticText4', parent=self, pos=wxPoint(120, 56),
              size=wxSize(12, 16), style=0)
        self.staticText4.SetFont(wxFont(12, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.text_g_end = wxTextCtrl(id=wxID_GRIDDIALOGTEXT_G_END,
              name='text_g_end', parent=self, pos=wxPoint(144, 56),
              size=wxSize(48, 22), style=0, value='')
        self.text_g_end.SetFont(wxFont(10, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.staticText5 = wxStaticText(id=wxID_GRIDDIALOGSTATICTEXT5,
              label='step', name='staticText5', parent=self, pos=wxPoint(200,
              20), size=wxSize(25, 16), style=0)
        self.staticText5.SetFont(wxFont(12, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.staticText6 = wxStaticText(id=wxID_GRIDDIALOGSTATICTEXT6,
              label='step', name='staticText6', parent=self, pos=wxPoint(200,
              56), size=wxSize(25, 16), style=0)
        self.staticText6.SetFont(wxFont(12, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.text_c_step = wxTextCtrl(id=wxID_GRIDDIALOGTEXT_C_STEP,
              name='text_c_step', parent=self, pos=wxPoint(232, 16),
              size=wxSize(48, 22), style=0, value='')
        self.text_c_step.SetFont(wxFont(10, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.text_g_step = wxTextCtrl(id=wxID_GRIDDIALOGTEXT_G_STEP,
              name='text_g_step', parent=self, pos=wxPoint(232, 56),
              size=wxSize(48, 22), style=0, value='')
        self.text_g_step.SetFont(wxFont(10, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.button_ok = wxButton(id=wxID_GRIDDIALOGBUTTON_OK, label='OK',
              name='button_ok', parent=self, pos=wxPoint(128, 96),
              size=wxSize(72, 24), style=0)
        EVT_BUTTON(self.button_ok, wxID_GRIDDIALOGBUTTON_OK,
              self.OnButton_okButton)

        self.button_cancel = wxButton(id=wxID_GRIDDIALOGBUTTON_CANCEL,
              label='Cancel', name='button_cancel', parent=self,
              pos=wxPoint(216, 96), size=wxSize(72, 24), style=0)
        EVT_BUTTON(self.button_cancel, wxID_GRIDDIALOGBUTTON_CANCEL,
              self.OnButton_cancelButton)

        self.text_fold = wxTextCtrl(id=wxID_GRIDDIALOGTEXT_FOLD,
              name='text_fold', parent=self, pos=wxPoint(64, 96),
              size=wxSize(48, 22), style=0, value='')
        self.text_fold.SetFont(wxFont(10, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.staticText7 = wxStaticText(id=wxID_GRIDDIALOGSTATICTEXT7,
              label='fold:', name='staticText7', parent=self, pos=wxPoint(24,
              96), size=wxSize(29, 16), style=0)
        self.staticText7.SetFont(wxFont(12, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

    def __init__(self, parent, config):
        self._init_ctrls(parent)
        self.config = config
        self.all_params = ['c_begin','c_end','c_step',
                           'g_begin','g_end','g_step',
                           'fold']
        self.read_config()
    
    def read_config(self):
        c = self.config
        for param in self.all_params:
            getattr(self,'text_'+param).SetValue(str(getattr(c,param)))
    
    def write_config(self):
        c = self.config
        for param in self.all_params:
            val = float(getattr(self,'text_'+param).GetValue())
            try:
                if val == int(val):
                    val = int(val)
            except:
                pass
            setattr(c,param,val)

    def OnButton_okButton(self, event):
        self.write_config()
        self.EndModal(True)
        
    def OnButton_cancelButton(self, event):        
        self.EndModal(False)
            