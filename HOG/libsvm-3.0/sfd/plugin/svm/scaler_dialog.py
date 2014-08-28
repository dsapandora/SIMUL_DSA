#Boa:Dialog:ScalerDialog

from wxPython.wx import *

def create(parent,config):
    return ScalerDialog(parent,config)

[wxID_SCALERDIALOG, wxID_SCALERDIALOGBUTTON_CANCEL, 
 wxID_SCALERDIALOGBUTTON_OK, wxID_SCALERDIALOGCHECKBOX1, 
 wxID_SCALERDIALOGSTATICTEXT1, wxID_SCALERDIALOGSTATICTEXT2, 
 wxID_SCALERDIALOGSTATICTEXT3, wxID_SCALERDIALOGTEXT_LOWER, 
 wxID_SCALERDIALOGTEXT_UPPER, wxID_SCALERDIALOGTEXT_Y_LOWER, 
 wxID_SCALERDIALOGTEXT_Y_UPPER, 
] = map(lambda _init_ctrls: wxNewId(), range(11))

class ScalerDialog(wxDialog):
    def _init_utils(self):
        # generated method, don't edit
        pass

    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wxDialog.__init__(self, id=wxID_SCALERDIALOG, name='ScalerDialog',
              parent=prnt, pos=wxPoint(226, 228), size=wxSize(296, 169),
              style=wxDEFAULT_DIALOG_STYLE, title='Scaler')
        self._init_utils()
        self.SetClientSize(wxSize(288, 142))

        self.staticText1 = wxStaticText(id=wxID_SCALERDIALOGSTATICTEXT1,
              label='Attribute range:', name='staticText1', parent=self,
              pos=wxPoint(32, 16), size=wxSize(74, 13), style=0)
        self.staticText1.SetFont(wxFont(10, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.text_y_lower = wxTextCtrl(id=wxID_SCALERDIALOGTEXT_Y_LOWER,
              name='text_y_lower', parent=self, pos=wxPoint(136, 52),
              size=wxSize(40, 24), style=0, value='-1')
        self.text_y_lower.Enable(false)

        self.text_y_upper = wxTextCtrl(id=wxID_SCALERDIALOGTEXT_Y_UPPER,
              name='text_y_upper', parent=self, pos=wxPoint(216, 52),
              size=wxSize(40, 24), style=0, value='1')
        self.text_y_upper.Enable(false)

        self.staticText2 = wxStaticText(id=wxID_SCALERDIALOGSTATICTEXT2,
              label='to', name='staticText2', parent=self, pos=wxPoint(192, 18),
              size=wxSize(9, 13), style=0)
        self.staticText2.SetFont(wxFont(10, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.button_ok = wxButton(id=wxID_SCALERDIALOGBUTTON_OK, label='OK',
              name='button_ok', parent=self, pos=wxPoint(40, 104),
              size=wxSize(87, 24), style=0)
        EVT_BUTTON(self.button_ok, wxID_SCALERDIALOGBUTTON_OK,
              self.OnButton_okButton)

        self.button_cancel = wxButton(id=wxID_SCALERDIALOGBUTTON_CANCEL,
              label='Cancel', name='button_cancel', parent=self,
              pos=wxPoint(168, 104), size=wxSize(87, 24), style=0)
        EVT_BUTTON(self.button_cancel, wxID_SCALERDIALOGBUTTON_CANCEL,
              self.OnButton_cancelButton)

        self.checkBox1 = wxCheckBox(id=wxID_SCALERDIALOGCHECKBOX1,
              label='Scale Y range:', name='checkBox1', parent=self,
              pos=wxPoint(32, 56), size=wxSize(88, 12), style=0)
        self.checkBox1.SetValue(false)
        EVT_CHECKBOX(self.checkBox1, wxID_SCALERDIALOGCHECKBOX1,
              self.OnCheckbox1Checkbox)

        self.text_lower = wxTextCtrl(id=wxID_SCALERDIALOGTEXT_LOWER,
              name='text_lower', parent=self, pos=wxPoint(136, 12),
              size=wxSize(40, 24), style=0, value='-1')

        self.staticText3 = wxStaticText(id=wxID_SCALERDIALOGSTATICTEXT3,
              label='to', name='staticText3', parent=self, pos=wxPoint(192, 59),
              size=wxSize(9, 13), style=0)
        self.staticText3.SetFont(wxFont(10, wxSWISS, wxNORMAL, wxNORMAL, false,
              '\xb7s\xb2\xd3\xa9\xfa\xc5\xe9'))

        self.text_upper = wxTextCtrl(id=wxID_SCALERDIALOGTEXT_UPPER,
              name='text_upper', parent=self, pos=wxPoint(216, 12),
              size=wxSize(40, 24), style=0, value='1')

    def __init__(self, parent,config):
        self._init_ctrls(parent)
        self.config = config
        self.read_config()
        
    def OnCheckbox1Checkbox(self, event):
        checked = bool(event.Checked())        
        self.text_y_lower.Enable(checked)
        self.text_y_upper.Enable(checked)
    
    def read_config(self):
        c = self.config
        self.text_lower.SetValue(str(c.lower))
        self.text_upper.SetValue(str(c.upper))
        if c.y_lower is None:
            self.text_y_lower.SetValue('-1')
            self.text_y_upper.SetValue('1')
            checked = False
        else:
            self.text_y_lower.SetValue(str(c.y_lower))
            self.text_y_upper.SetValue(str(c.y_upper))            
            checked = True
        self.checkBox1.SetValue(checked)
        self.text_y_lower.Enable(checked)
        self.text_y_upper.Enable(checked)
        
    def write_config(self):
        c = self.config
        c.lower = float(self.text_lower.GetValue())
        c.upper = float(self.text_upper.GetValue())
        if self.checkBox1.GetValue():
            c.y_lower = float(self.text_y_lower.GetValue())
            c.y_upper = float(self.text_y_upper.GetValue())        
        else:
            c.y_lower = None
            c.y_upper = None

    def OnButton_okButton(self, event):
        self.write_config()
        self.EndModal(True)
                
    def OnButton_cancelButton(self, event):                
        self.EndModal(False)
