#Boa:Dialog:TrainerDialog

from wxPython.wx import *

def create(parent,config):
    return TrainerDialog(parent,config)

[wxID_TRAINERDIALOG, wxID_TRAINERDIALOGBUTTON_CANCEL, 
 wxID_TRAINERDIALOGBUTTON_OK, wxID_TRAINERDIALOGLABEL_CACHE_SIZE, 
 wxID_TRAINERDIALOGLABEL_COEF0, wxID_TRAINERDIALOGLABEL_COST, 
 wxID_TRAINERDIALOGLABEL_DEGREE, wxID_TRAINERDIALOGLABEL_FOLD, 
 wxID_TRAINERDIALOGLABEL_GAMMA, wxID_TRAINERDIALOGLABEL_NU, 
 wxID_TRAINERDIALOGLABEL_REGRESSION_EPSILON, 
 wxID_TRAINERDIALOGLABEL_SHRINKING, wxID_TRAINERDIALOGLABEL_STOPPING_EPSILON, 
 wxID_TRAINERDIALOGPANEL1, wxID_TRAINERDIALOGRADIO_KERNEL_TYPE, 
 wxID_TRAINERDIALOGRADIO_SVM_TYPE, wxID_TRAINERDIALOGSTATICBOX1, 
 wxID_TRAINERDIALOGTEXT_CACHE_SIZE, wxID_TRAINERDIALOGTEXT_COEF0, 
 wxID_TRAINERDIALOGTEXT_COST, wxID_TRAINERDIALOGTEXT_DEGREE, 
 wxID_TRAINERDIALOGTEXT_FOLD, wxID_TRAINERDIALOGTEXT_GAMMA, 
 wxID_TRAINERDIALOGTEXT_NU, wxID_TRAINERDIALOGTEXT_REGRESSION_EPSILON, 
 wxID_TRAINERDIALOGTEXT_SHRINKING, wxID_TRAINERDIALOGTEXT_STOPPING_EPSILON, 
] = map(lambda _init_ctrls: wxNewId(), range(27))

class TrainerDialog(wxDialog):
    def _init_utils(self):
        # generated method, don't edit
        pass

    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wxDialog.__init__(self, id=wxID_TRAINERDIALOG, name='TrainerDialog',
              parent=prnt, pos=wxPoint(290, 246), size=wxSize(548, 327),
              style=wxDEFAULT_DIALOG_STYLE, title='Trainer')
        self._init_utils()
        self.SetClientSize(wxSize(548, 327))

        self.panel1 = wxPanel(id=wxID_TRAINERDIALOGPANEL1, name='panel1',
              parent=self, pos=wxPoint(0, 0), size=wxSize(548, 327),
              style=wxTAB_TRAVERSAL)

        self.radio_svm_type = wxRadioBox(choices=['C-SVC', 'nu-SVC',
              'one-class SVM', 'epsilon-SVR', 'nu-SVR'],
              id=wxID_TRAINERDIALOGRADIO_SVM_TYPE, label='SVM Type',
              majorDimension=1, name='radio_svm_type', parent=self.panel1,
              point=wxPoint(10, 8), size=wxSize(270, 144),
              style=wxRA_SPECIFY_COLS, validator=wxDefaultValidator)
        self.radio_svm_type.SetStringSelection('C-SVC')
        EVT_RADIOBOX(self.radio_svm_type, wxID_TRAINERDIALOGRADIO_SVM_TYPE,
              self.OnRadio_svm_typeRadiobox)

        self.radio_kernel_type = wxRadioBox(choices=["linear: u'*v",
              "polynomial: (gamma*u'*v + coef0)^degree",
              "radial basis function: exp(-gamma*|u-v|^2)",
              "sigmoid: tanh(gamma*u'*v + coef0)"],
              id=wxID_TRAINERDIALOGRADIO_KERNEL_TYPE, label='Kernel Type',
              majorDimension=1, name='radio_kernel_type', parent=self.panel1,
              point=wxPoint(10, 177), size=wxSize(270, 136),
              style=wxRA_SPECIFY_COLS, validator=wxDefaultValidator)
        self.radio_kernel_type.SetStringSelection('radial basis function: exp(-gamma*|u-v|^2)')
        EVT_RADIOBOX(self.radio_kernel_type,
              wxID_TRAINERDIALOGRADIO_KERNEL_TYPE,
              self.OnRadio_kernel_typeRadiobox)

        self.label_gamma = wxStaticText(id=wxID_TRAINERDIALOGLABEL_GAMMA,
              label='gamma', name='label_gamma', parent=self.panel1,
              pos=wxPoint(312, 76), size=wxSize(56, 16), style=0)

        self.label_cost = wxStaticText(id=wxID_TRAINERDIALOGLABEL_COST,
              label='cost', name='label_cost', parent=self.panel1,
              pos=wxPoint(312, 28), size=wxSize(40, 16), style=0)

        self.label_degree = wxStaticText(id=wxID_TRAINERDIALOGLABEL_DEGREE,
              label='degree', name='label_degree', parent=self.panel1,
              pos=wxPoint(312, 100), size=wxSize(56, 16), style=0)

        self.label_nu = wxStaticText(id=wxID_TRAINERDIALOGLABEL_NU, label='nu',
              name='label_nu', parent=self.panel1, pos=wxPoint(312, 52),
              size=wxSize(40, 16), style=0)

        self.label_coef0 = wxStaticText(id=wxID_TRAINERDIALOGLABEL_COEF0,
              label='coef0', name='label_coef0', parent=self.panel1,
              pos=wxPoint(312, 124), size=wxSize(48, 16), style=0)

        self.label_cache_size = wxStaticText(id=wxID_TRAINERDIALOGLABEL_CACHE_SIZE,
              label='cachesize', name='label_cache_size', parent=self.panel1,
              pos=wxPoint(312, 148), size=wxSize(56, 16), style=0)

        self.label_shrinking = wxStaticText(id=wxID_TRAINERDIALOGLABEL_SHRINKING,
              label='shrinking', name='label_shrinking', parent=self.panel1,
              pos=wxPoint(312, 172), size=wxSize(64, 16), style=0)

        self.label_regression_epsilon = wxStaticText(id=wxID_TRAINERDIALOGLABEL_REGRESSION_EPSILON,
              label='regression_epsilon', name='label_regression_epsilon',
              parent=self.panel1, pos=wxPoint(312, 220), size=wxSize(104, 16),
              style=0)

        self.label_fold = wxStaticText(id=wxID_TRAINERDIALOGLABEL_FOLD,
              label='cross_validation_fold', name='label_fold',
              parent=self.panel1, pos=wxPoint(312, 244), size=wxSize(120, 16),
              style=0)

        self.label_stopping_epsilon = wxStaticText(id=wxID_TRAINERDIALOGLABEL_STOPPING_EPSILON,
              label='stopping_epsilon', name='label_stopping_epsilon',
              parent=self.panel1, pos=wxPoint(312, 196), size=wxSize(96, 16),
              style=0)

        self.staticBox1 = wxStaticBox(id=wxID_TRAINERDIALOGSTATICBOX1,
              label='Options', name='staticBox1', parent=self.panel1,
              pos=wxPoint(296, 8), size=wxSize(232, 264), style=0)

        self.text_cost = wxTextCtrl(id=wxID_TRAINERDIALOGTEXT_COST,
              name='text_cost', parent=self.panel1, pos=wxPoint(384, 24),
              size=wxSize(128, 20), style=0, value='1')

        self.text_nu = wxTextCtrl(id=wxID_TRAINERDIALOGTEXT_NU, name='text_nu',
              parent=self.panel1, pos=wxPoint(384, 48), size=wxSize(128, 20),
              style=0, value='0.5')

        self.text_gamma = wxTextCtrl(id=wxID_TRAINERDIALOGTEXT_GAMMA,
              name='text_gamma', parent=self.panel1, pos=wxPoint(384, 72),
              size=wxSize(128, 20), style=0, value='0')

        self.text_degree = wxTextCtrl(id=wxID_TRAINERDIALOGTEXT_DEGREE,
              name='text_degree', parent=self.panel1, pos=wxPoint(384, 96),
              size=wxSize(128, 20), style=0, value='3')

        self.text_coef0 = wxTextCtrl(id=wxID_TRAINERDIALOGTEXT_COEF0,
              name='text_coef0', parent=self.panel1, pos=wxPoint(384, 120),
              size=wxSize(128, 20), style=0, value='0')

        self.text_cache_size = wxTextCtrl(id=wxID_TRAINERDIALOGTEXT_CACHE_SIZE,
              name='text_cache_size', parent=self.panel1, pos=wxPoint(384, 144),
              size=wxSize(128, 20), style=0, value='40')

        self.text_shrinking = wxTextCtrl(id=wxID_TRAINERDIALOGTEXT_SHRINKING,
              name='text_shrinking', parent=self.panel1, pos=wxPoint(384, 168),
              size=wxSize(128, 20), style=0, value='1')

        self.text_stopping_epsilon = wxTextCtrl(id=wxID_TRAINERDIALOGTEXT_STOPPING_EPSILON,
              name='text_stopping_epsilon', parent=self.panel1, pos=wxPoint(432,
              192), size=wxSize(80, 20), style=0, value='0.001')

        self.text_regression_epsilon = wxTextCtrl(id=wxID_TRAINERDIALOGTEXT_REGRESSION_EPSILON,
              name='text_regression_epsilon', parent=self.panel1,
              pos=wxPoint(432, 216), size=wxSize(80, 20), style=0, value='0.1')

        self.text_fold = wxTextCtrl(id=wxID_TRAINERDIALOGTEXT_FOLD,
              name='text_fold', parent=self.panel1, pos=wxPoint(432, 240),
              size=wxSize(80, 22), style=0, value='')

        self.button_ok = wxButton(id=wxID_TRAINERDIALOGBUTTON_OK, label='OK',
              name='button_ok', parent=self.panel1, pos=wxPoint(320, 288),
              size=wxSize(88, 24), style=0)
        EVT_BUTTON(self.button_ok, wxID_TRAINERDIALOGBUTTON_OK,
              self.OnButton_okButton)

        self.button_cancel = wxButton(id=wxID_TRAINERDIALOGBUTTON_CANCEL,
              label='Cancel', name='button_cancel', parent=self.panel1,
              pos=wxPoint(440, 288), size=wxSize(87, 24), style=0)
        EVT_BUTTON(self.button_cancel, wxID_TRAINERDIALOGBUTTON_CANCEL,
              self.OnButton_cancelButton)

    def __init__(self, parent, config):
        self._init_ctrls(parent)
        self.all_names = [ 'cost', 'nu', 'gamma', 'degree','coef0', 'cache_size'
                          ,'shrinking', 'stopping_epsilon','regression_epsilon',
                          'fold' ]
        self.name2label = dict([(name,getattr(self,'label_'+name)) 
                            for name in self.all_names])        
        self.name2text = dict([(name,getattr(self,'text_'+name)) 
                            for name in self.all_names])
        self.config = config
        self.read_config()
        self.grey_out_check()

    def read_config(self):
        c = self.config
        self.radio_svm_type.SetSelection(c.svm_type)
        self.radio_kernel_type.SetSelection(c.kernel_type)
        for name in self.all_names:
            getattr(self,'text_'+name).SetValue(str(getattr(c,name)))

    def write_config(self):
        c = self.config
        c.svm_type = self.radio_svm_type.GetSelection()
        c.kernel_type = self.radio_kernel_type.GetSelection()
        for name in self.all_names:
            val = float(getattr(self,'text_'+name).GetValue())
            try:
                if val == int(val):
                    val = int(val)
            except:
                pass
            setattr(c,name,val)

    def grey_out_check(self):
        # according to svm_type and kernel_type, grey out some options
        
        for name in self.all_names:
            self.name2label[name].Enable(False)
            self.name2text[name].Enable(False)
            
        enable_list = ['cache_size','stopping_epsilon','shrinking','fold']

        svm_type = self.radio_svm_type.GetSelection()
        
        if svm_type == 0:
            enable_list.extend(['cost'])
        elif svm_type == 1:
            enable_list.extend(['nu'])
        elif svm_type == 2:
            enable_list.extend(['nu'])
        elif svm_type == 3:
            enable_list.extend(['regression_epsilon','cost'])
        elif svm_type == 4:
            enable_list.extend(['nu','cost'])
        
        kernel_type = self.radio_kernel_type.GetSelection()

        if kernel_type == 0:
            enable_list.extend([])
        elif kernel_type == 1:
            enable_list.extend(['degree','gamma','coef0'])
        elif kernel_type == 2:
            enable_list.extend(['gamma'])
        elif kernel_type == 3:
            enable_list.extend(['gamma','coef0'])

        for name in enable_list:
            self.name2label[name].Enable(True)
            self.name2text[name].Enable(True)

    def OnRadio_svm_typeRadiobox(self, event):
        self.grey_out_check()
        
    def OnRadio_kernel_typeRadiobox(self, event):
        self.grey_out_check()

    def OnButton_okButton(self, event):
        self.write_config()
        self.EndModal(True)

    def OnButton_cancelButton(self, event):
        self.EndModal(False)
